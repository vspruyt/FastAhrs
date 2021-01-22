#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include "CalibrationEEPROMwriter.h"

/*
  Wite sensor calibration parameters to EEPROM
*/

CalibrationEEPROMwriter calibration_writer;

// Set to false if you only want to read and print the current calibration,
// from EEPROM, but don't want to write any new calibration data to EEPROM.
#define WRITE (false)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(100);
  calibration_writer.begin();
  
  SensorCalibration calibration_data;

  if (!calibration_writer.loadCalibration(&calibration_data)) {
    Serial.println("No calibration loaded/found... will start with defaults");
  } else {
    Serial.println("Loaded existing calibration");
  }

  if(WRITE){
    // MAGNETOMETER
    // -------------
    calibration_data.mag_offset = {-10.2737655640, -13.2165718079, 62.1393280029};
    calibration_data.mag_scaling = {0.9826576710, -0.0288685262, -0.0057038367, 
                                    -0.0288685262, 0.9941087365, 0.0126562417, 
                                    -0.0057038367, 0.0126562417, 1.0247431993};

    // GYROSCOPE
    // ---------
    calibration_data.gyro_offset = {0.0047237240, 0.0602602996, 0.0035765988};
    
    // ACCELEROMETER
    // --------------
    calibration_data.accel_offset = {0.2267748645331869, -0.3271435962754346, 0.5619391987862744};
    calibration_data.accel_scaling = {1.0022437247616607, -0.0011055495003400868, -0.00221625733452418, 
                                      -0.0011149309128299049, 1.0110417470405177, 0.0018268302812565944, 
                                      -0.002221837016460909, 0.0018159538076822795, 1.004761996085182};
    

    // Save the calibration data to EEPROM
    if (! calibration_writer.saveCalibration(calibration_data)) {
      Serial.println("**WARNING** Couldn't save calibration");
    } else {
      Serial.println("Wrote calibration");    
    }
  }

  // Load the calibration data back from EEPROM
  calibration_writer.loadCalibration(&calibration_data);

  // Print out the calibration data
  Serial.println("Calibrations read back from EEPROM: ");
  Serial.print("\tMagnetic Hard Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(calibration_data.mag_offset.offset[i], 10); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();
  
  Serial.print("\tMagnetic Soft Offset: ");
  for (int i=0; i<9; i++) {
    Serial.print(calibration_data.mag_scaling.scaling[i], 10); 
    if (i != 8) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tGyro Zero Rate Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(calibration_data.gyro_offset.offset[i], 10); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tAccel Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(calibration_data.accel_offset.offset[i], 10); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tAccel Scaling: ");
  for (int i=0; i<9; i++) {
    Serial.print(calibration_data.accel_scaling.scaling[i], 10); 
    if (i != 8) Serial.print(", ");
  }  
  Serial.println();
}

void loop() {

}