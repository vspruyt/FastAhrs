// Robust 400Hz Atitude and Heading (AHRS) estimation for the AdaFruit 
// "Precision NXP 9-DOF breakout board - FXOS8700 + FXAS21002"
// This library solves a bunch of issues that come with the default AdaFruit
// libraries, and uses the latest updates of the MadgWick filter that few people 
// seem to be aware of.

// This implementation runs at 400Hz on a Teensy 4.0 microcontroller.
// It contains:
// - Sensor fusion (accelerometer, gyroscope, magnetometer) for attitude estimation
// - Outlier removal for magnetometer values to deal with environmental factors
// - Outlier removal for accelerometer values to deal with fast motion
// - Advanced accelerometer calibration (12 parameters; 9 for scaling, 3 for offsets)
// - Support to drive pin output for indicator leds showing anomaly and initialization status
// - Boiler plate code to write and read calibration to/from EEPROM on the Teensy

// To use this, you must calibrate the sensors, and write the calibration parameters
// to EEPROM. Check the calibrate_accel, calibrate_gyro and calibrate_magnetometer samples
// in the 'examples' folder to obtain the calibration parameters.
// You can use the 'CalibrationEEPROM_read_write.cpp' file to write the values to EEPROM.

// The underlying attitude estimation uses the Madgwick complementary filter.
// Note: This is not based on the older gradient based Madgwick filter that lots of people 
// still use! Instead, it's based on the latest version of his attitude estimator that 
// Madgwick himself recommends: https://github.com/xioTechnologies/Fusion
// In Sebastian Madgwick's PhD thesis, titled "AHRS algorithms and calibration solutions to
// facilitate new applications using low-cost MEMS", he first discusses the gradient 
// based filter that most people know about. At the end of his thesis, he then discusses his
// 'revised algorithm', which offers several improvements upon the original algorithm,
// is much more efficient and robust against bad sensor readings from typical low-cost sensors,
// and is similar to the Mahoney algorithm.

// Note: The AdaFruit default AHRS libraries only run at 100Hz by default, and do
// not support full 12DOF accelerometer calibration.  Also, the AdaFruit sensor 
// abstractions are  not very efficient when used for the precision NXP breakout-board, 
// because the abstractions force you to query accelerometer and magnetometer in separate 
// calls, while they can and should be polled in a single instruction. 
// This library solves all of that.

// Finally, the default AdaFruit implementation configures the gyroscope for a 250dps 
// sensitivity. However, this is not enough if you want to cope with fast motion 
// (e.g. shaking the sensor by hand). In this implementation, we configure the gyroscope 
// for a 1000dps sensitivity instead.

// To correctly run at 400 HZ, you will have to make the following changes to the
// Adafruit sensor drivers that are used:
// Adafruit_FXOS8700.cpp: Line 184 (use 0x15 for 100Hz, 0x05 for 400Hz)
// Adafruit_FXAS21002C.cpp: Line 203 (use 0x0E for 100Hz, 0x06 for 400Hz)

#include <Wire.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Fusion.h>
#include "CalibrationEEPROMwriter.h"

// Baud rate for output serial where we write the quaternion that represents the 
// sensor orientation.
// In total, we write:
// - 2 header bytes to indicate start of transmission
// - 4 quaternion floats, each represented by 4 bytes 
// - Total: 4*4+2=18 bytes 
// This corresponds to 18*8=144 bits, which we want to send over 400 times per second.
// Hence, we need a baud rate of at least 144*400=57.6kbps
#define OUTPUT_BAUDRATE 115200
#define OUTPUT_SERIAL Serial5

// Configure the indicator led output pins
#define MAG_INITIALIZED_LED_OUTPUT_PIN 11
#define MAG_BAD_MAGNITUDE_LED_OUTPUT_PIN 15
#define MAG_BAD_VARIANCE_LED_OUTPUT_PIN 10
#define ACC_BAD_MAGNITUDE_LED_OUTPUT_PIN 7

// To use an update rate of 400 Hz, you have to change:
// Adafruit_FXOS8700.cpp: Line 184 (use 0x15 for 100Hz, 0x05 for 400Hz)
// Adafruit_FXAS21002C.cpp: Line 203 (use 0x0E for 100Hz, 0x06 for 400Hz)
// To find these files, just cmd-click (or ctrl-click) on the include 
// statements at the top of this file, which will bring you to the folder
// that contains the two cpp files.
//
// Note: Don't just change this constant to a different frequency! You have to
// Update the registers of the sensors too if you want a different sampling frequency,
// by changing the two files mentioned.
// If you change this constant to 100 instead of 400, then you don't have to change any 
// files, but the filter will only run at 100Hz instead of 400Hz.
#define FILTER_UPDATE_RATE_HZ 400
#define PRINT_EVERY_N_UPDATES 40

// Defines the size (in number of seconds) of the rolling window for 
// magnetometer outlier removal
#define ANOMALY_REJECTION_WINDOW_SIZE_S (10)

#include "SensorAnomalyRejector.h"

Adafruit_FXOS8700 imu_accel_mag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C imu_gyroscope = Adafruit_FXAS21002C(0x0021002C);
sensors_event_t accel, gyro, mag;

FusionBias fusionBias;
FusionAhrs fusionAhrs;

// Outlier detection. We consider the magntic field an anomaly if its magnitude
// deviates more than 5.0 uT from its running average over the past 10 seconds,
// or if its running standard deviation is higher than 3uT. Assuming a normal distribution,
// the latter case would mean that about 30% of the measurements are more than 3uT away 
// from it's running average, while about 5% deviates even more than 6uT. This indicates
// a fluctuating, unstable magnetic field that we want to reject.
// We set the replacement value for anomalies to 0.0f, which is ignored by our sensor
// fusion filter.
// Note: Independent of this logic, the filter also rejects any non-sensical magnetic 
// field magnitude, defined later in this code such that anything outside the range
// of [23uT, 67uT] also gets rejected.
SensorAnomalyRejector magnetometerValidator(5.0f, 3.0f, 0.0f);

CalibrationEEPROMwriter calibration_writer;
SensorCalibration calibration_data;

float samplePeriod = 1.0f/FILTER_UPDATE_RATE_HZ;
uint32_t timestamp;

bool init_sensors(void) {
  // We set the gyro range to 1000dps instead of the Adafruit default of 250dps, to cope with fast motion.
  // If you manually shake the IMU around, you'll notice that 250 or even 500 dps is not sensitive enough,
  // causing errors and a slow recovery, while 1000dps covers most of those cases.
  // You can go higher (up to 2000dps), but that reduces the sensitivity proportionally. So at 500dps,
  // the gyroscope is half as sensitive as at 250dps. It is best to chose the range as low as possible,
  // depending on the application requirements.
  // Also, we don't use the Adafruit_Sensor abstraction because that forces us to ask for accelerometer
  // and gyroscope samples separately using the individual getEvent() methods. Instead, we can now
  // ask for both gyroscope and accelerometer in one call, making sure that the values we receive
  // are as close in time as possible.
  if (!imu_accel_mag.begin(ACCEL_RANGE_2G) || !imu_gyroscope.begin(GYRO_RANGE_1000DPS)) {
    return false;
  }  

  return true;
}

void calibrate_sensors(SensorCalibration& calibration_data, sensors_event_t& gyro, sensors_event_t& accel, sensors_event_t& mag){
  FusionVector3* calib_accel_offset = (FusionVector3*) &(calibration_data.accel_offset.offset);
  FusionRotationMatrix* calib_accel_scaling = (FusionRotationMatrix*) &(calibration_data.accel_scaling);  
  FusionVector3* calib_gyro_offset = (FusionVector3*) &(calibration_data.gyro_offset.offset);
  FusionVector3* calib_mag_offset = (FusionVector3*) &(calibration_data.mag_offset.offset);
  FusionRotationMatrix* calib_mag_scaling = (FusionRotationMatrix*) &(calibration_data.mag_scaling);

  FusionVector3 calibrated_gyro = FusionCalibrationInertial({gyro.gyro.x, gyro.gyro.y, gyro.gyro.z}, FUSION_ROTATION_MATRIX_IDENTITY, {1.0f, 1.0f, 1.0f}, *calib_gyro_offset);
  FusionVector3 calibrated_accel = FusionCalibrationInertial({accel.acceleration.x, accel.acceleration.y, accel.acceleration.z}, *calib_accel_scaling, {1.0f, 1.0f, 1.0f}, *calib_accel_offset);
  FusionVector3 calibrated_mag = FusionCalibrationInertial({mag.magnetic.x, mag.magnetic.y, mag.magnetic.z}, *calib_mag_scaling, {1.0f, 1.0f, 1.0f}, *calib_mag_offset);  
  
  gyro.gyro.x = calibrated_gyro.axis.x;
  gyro.gyro.y = calibrated_gyro.axis.y;
  gyro.gyro.z = calibrated_gyro.axis.z;

  accel.acceleration.x = calibrated_accel.axis.x;
  accel.acceleration.y = calibrated_accel.axis.y;
  accel.acceleration.z = calibrated_accel.axis.z;

  mag.magnetic.x = calibrated_mag.axis.x;
  mag.magnetic.y = calibrated_mag.axis.y;
  mag.magnetic.z = calibrated_mag.axis.z;
}

FusionVector3 validate_acceleration(float x, float y, float z, float threshold){
  FusionVector3 validaccel = {x, y, z};
  float mag = x*x + y*y + z*z;
  if(mag > threshold*threshold){
      validaccel.axis.x = 0.0;
      validaccel.axis.y = 0.0;
      validaccel.axis.z = 0.0;
  }
  return validaccel;
}

void set_indicator_leds(const SensorAnomalyRejector& magnetometerValidator, const FusionVector3& validaccel){
    digitalWrite(MAG_INITIALIZED_LED_OUTPUT_PIN, magnetometerValidator.is_initialized);      
    digitalWrite(MAG_BAD_MAGNITUDE_LED_OUTPUT_PIN, magnetometerValidator.is_rejecting_magnitude);      
    digitalWrite(MAG_BAD_VARIANCE_LED_OUTPUT_PIN, magnetometerValidator.is_rejecting_variance);  
    digitalWrite(ACC_BAD_MAGNITUDE_LED_OUTPUT_PIN, validaccel.axis.x==0 && validaccel.axis.y==0 && validaccel.axis.z==0);
}

void setup() {

  // Configure the output serial port
  OUTPUT_SERIAL.begin(OUTPUT_BAUDRATE);  

  // Initialize the LED indicators
  pinMode(MAG_INITIALIZED_LED_OUTPUT_PIN, OUTPUT);
  digitalWrite(MAG_INITIALIZED_LED_OUTPUT_PIN, LOW);

  pinMode(MAG_BAD_MAGNITUDE_LED_OUTPUT_PIN, OUTPUT);
  digitalWrite(MAG_BAD_MAGNITUDE_LED_OUTPUT_PIN, LOW);  

  pinMode(MAG_BAD_VARIANCE_LED_OUTPUT_PIN, OUTPUT);
  digitalWrite(MAG_BAD_VARIANCE_LED_OUTPUT_PIN, LOW);  

  pinMode(ACC_BAD_MAGNITUDE_LED_OUTPUT_PIN, OUTPUT);
  digitalWrite(ACC_BAD_MAGNITUDE_LED_OUTPUT_PIN, LOW);

  Serial.begin(115200);

  if (!calibration_writer.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! calibration_writer.loadCalibration(&calibration_data)) {
    Serial.println("No calibration loaded/found");
  }  

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  imu_accel_mag.getAccelerometerSensor()->printSensorDetails();
  imu_accel_mag.getMagnetometerSensor()->printSensorDetails();
  imu_gyroscope.printSensorDetails();  
  
  timestamp = micros();  

  Wire.setClock(400000); // 400KHz

  // Initialise gyroscope bias correction algorithm
  FusionBiasInitialise(&fusionBias, 2.0f, samplePeriod); // stationary threshold = 2 degrees per second

  // Initialise AHRS algorithm
  FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5

  // Set optional magnetic field limits  
  // valid magnetic field on Earth: 25 uT to 65 uT, plus some margin to deal with noise 
  // (e.g. our magnetometer has a 0.8 uT/C temperature dependent offset)
  FusionAhrsSetMagneticField(&fusionAhrs, 23.0f, 67.0f); 
}

void loop() {  

  // Timing stuff to keep a constant 400Hz sample rate
  static uint8_t counter = 0;  
  uint32_t curr_time = micros();
  uint32_t time_diff = curr_time - timestamp;
  if (time_diff < (1000000 / FILTER_UPDATE_RATE_HZ)) return;
  samplePeriod = time_diff/1000000.0;
  timestamp = curr_time;

  // Read the motion sensors  
  imu_gyroscope.getEvent(&gyro); 
  imu_accel_mag.getEvent(&accel, &mag);  

  // Sensor calibration  
  calibrate_sensors(calibration_data, gyro, accel, mag);  

  // Outlier removal
  FusionVector3 validmag = magnetometerValidator.update(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);  
  FusionVector3 validaccel = validate_acceleration(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 9.81*1.1);  
  FusionVector3 validgyro = {gyro.gyro.x * SENSORS_RADS_TO_DPS, gyro.gyro.y * SENSORS_RADS_TO_DPS, gyro.gyro.z * SENSORS_RADS_TO_DPS};

  // Update gyroscope bias correction algorithm
  FusionVector3 online_recalibrated_gyro = FusionBiasUpdate(&fusionBias, validgyro);  

  // Update AHRS algorithm
  FusionAhrsUpdate(&fusionAhrs, online_recalibrated_gyro, {validaccel.axis.x/9.81, validaccel.axis.y/9.81, validaccel.axis.z/9.81}, {validmag.axis.x, validmag.axis.y, validmag.axis.z}, samplePeriod);

  // Get the sensor orientation as a quaternion
  FusionQuaternion sensor_orientation = FusionAhrsGetQuaternion(&fusionAhrs);

  // Write the quaternion to the Serial output port
  OUTPUT_SERIAL.print("\001\002"); // Start-of-header and start-of-text bytes
  OUTPUT_SERIAL.write((uint8_t*) sensor_orientation.array, sizeof(sensor_orientation.array));  

  // Set the indicator leds
  set_indicator_leds(magnetometerValidator, validaccel);

  // only print the calculated output once in a while
  if (counter++ <= PRINT_EVERY_N_UPDATES)
    return;
  
  // reset the counter
  counter = 0;
  
  Serial.print("Orientation: ");
  FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(sensor_orientation);
  Serial.print(eulerAngles.angle.yaw);
  Serial.print(" ");
  Serial.print(eulerAngles.angle.pitch);
  Serial.print(" ");
  Serial.println(eulerAngles.angle.roll); 

  // FusionQuaternion quaternion = FusionAhrsGetQuaternion(&fusionAhrs);  
  // Serial.print("Quaternion: ");
  // Serial.print(quaternion.element.w, 5);
  // Serial.print(", ");
  // Serial.print(quaternion.element.x, 5);
  // Serial.print(", ");
  // Serial.print(quaternion.element.y, 5);
  // Serial.print(", ");
  // Serial.println(quaternion.element.z, 5);      

}