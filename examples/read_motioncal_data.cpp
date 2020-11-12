#include "Adafruit_Sensor_Calibration.h"

Adafruit_Sensor_Calibration_EEPROM cal;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(100);
  Serial.println("Calibration filesys test");
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1) yield();
  }
  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
  

  if (! cal.loadCalibration()) {
    Serial.println("**WARNING** No calibration loaded/found");
  }

  Serial.println("Calibrations found: ");
  Serial.print("\tMagnetic Hard Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.mag_hardiron[i], 10); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();
  
  Serial.print("\tMagnetic Soft Offset: ");
  for (int i=0; i<9; i++) {
    Serial.print(cal.mag_softiron[i], 10); 
    if (i != 8) Serial.print(", ");
  }
  Serial.println();

}

void loop() {

}