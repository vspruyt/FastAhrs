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
    calibration_data.mag_offset = {2.6268424988, -43.6149749756, -45.4754714966};
    calibration_data.mag_scaling = {0.9767574072, -0.0219166204, 0.0224733353,
                                    -0.0219166204, 0.9923714995, 0.0015654713,
                                    0.0224733353, 0.0015654713, 1.0326985121};

    // GYROSCOPE
    // ---------
    calibration_data.gyro_offset = {0.0123780277, -0.0247886013, -0.0040863520};
    
    // ACCELEROMETER
    // --------------
    calibration_data.accel_offset = {0.2309987896844486, -0.2825938619321421, 0.4864192677463331};
    calibration_data.accel_scaling = {1.014599384722039, 2.3229277236592188e-05, -0.0020536702796896205, 
                                      6.797192940542769e-06, 1.0197755632694088, 0.0004826774750067301, 
                                      -0.002026976887912054, 0.00047222849767767716, 1.000789918004966};
    

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