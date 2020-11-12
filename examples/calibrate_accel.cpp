/* 
  This code simply logs accelerometer values to Serial, which can be dumped to a file.
  E.g. if you use PlatformIO, you can dump the output by the following 
  command: 'pio device monitor | tee notebooks/data_sample.csv'.
  The output file can then be used to calibrate the accelerometer with the provided
  Python Jupyter notebook 'AccelerometerCalibration.ipynb'.  

  Values are logged for a specific time period (e.g. 10 seconds by default), after which
  you are asked to put the accelerometer in a different orientation, and start logging again.

  You should place the accelerometer in at least 9 different orientations. Six of those
  can be obtained by conceptually attaching the IMU to a cube, and placing the cube on each
  side for a period of time while logging data. More orientations can be obtained by conceptually
  sanding off the corners of the cube, and also placing the cube on those new sides.
  Ideally, many more unique positions are used, so go wild :).

  Note: You don't actually need a cube, just hold it steady in your hand or on a table :).
*/

#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor_Calibration.h>

#define NUMBER_OF_CALIBRATION_SECONDS (10)
#define UPDATE_RATE_HZ (400)

Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_Sensor *accelerometer;


uint32_t timestamp;
void setup() {
  Serial.begin(115200);
  while (!Serial) yield();  
  
  fxos.begin(ACCEL_RANGE_2G);
  accelerometer = fxos.getAccelerometerSensor();
  
  accelerometer->printSensorDetails();
    
  timestamp = micros();  

  Wire.setClock(400000); // 400KHz

  Serial.println("x, y, z");

  Serial.println(F(">>> Place accelerometer on flat, stable surface!"));
  
}


void loop() {
  
  Serial.println(">>> Type key when ready..."); 
  while (!Serial.available()){}  // wait for a character

  Serial.print(">>> Gathering data for "); 
  Serial.print(NUMBER_OF_CALIBRATION_SECONDS);
  Serial.println(" seconds..."); 

  for(int i=0; i<UPDATE_RATE_HZ*NUMBER_OF_CALIBRATION_SECONDS; ++i){

    uint32_t curr_time = micros();
    uint32_t time_diff = curr_time - timestamp;

    while (time_diff < (1000000 / UPDATE_RATE_HZ)) {
      yield();
      curr_time = micros();
      time_diff = curr_time - timestamp;
    }    
    timestamp = curr_time;
        
    sensors_event_t accelEvent;  
    accelerometer->getEvent(&accelEvent);

    Serial.print(accelEvent.acceleration.x, 10);
    Serial.print(", ");
    Serial.print(accelEvent.acceleration.y, 10);
    Serial.print(", ");
    Serial.println(accelEvent.acceleration.z, 10);    
  }

  while (Serial.available())
  {
    Serial.read();  // clear the input buffer
  }
}