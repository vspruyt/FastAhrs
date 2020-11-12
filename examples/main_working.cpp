#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <Fusion.h>

// FusionBias fusionBias;
// FusionAhrs fusionAhrs;

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void displaySensorDetails(void) {
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) {

  pinMode(18, INPUT);
  pinMode(19, INPUT);

  Serial.begin(9600);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  Serial.println("FXOS8700 Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accelmag.begin(ACCEL_RANGE_4G)) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void) {
  sensors_event_t aevent, mevent;

  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);

  /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.print("A ");
  Serial.print("X: ");
  Serial.print(aevent.acceleration.x, 4);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(aevent.acceleration.z, 4);
  Serial.print("  ");
  Serial.println("m/s^2");

  /* Display the mag results (mag data is in uTesla) */
  // Serial.print("M ");
  // Serial.print("X: ");
  // Serial.print(mevent.magnetic.x, 1);
  // Serial.print("  ");
  // Serial.print("Y: ");
  // Serial.print(mevent.magnetic.y, 1);
  // Serial.print("  ");
  // Serial.print("Z: ");
  // Serial.print(mevent.magnetic.z, 1);
  // Serial.print("  ");
  // Serial.println("uT");

  Serial.println("");

  delay(50);
}


// // Simple Example Sample
// // Copyright (c) 2012 Dimension Engineering LLC
// // See license.txt for license details.
// #include <SabertoothSimplified.h>
// #include <Arduino.h>
// #include <Encoder.h>

// SabertoothSimplified ST(Serial1); // We'll name the Sabertooth object ST.
//                          // For how to configure the Sabertooth, see the DIP Switch Wizard for
//                          //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
//                          // Be sure to select Simplified Serial Mode for use with this library.
//                          // This sample uses a baud rate of 9600.
//                          //
//                          // Connections to make:
//                          //   Arduino TX->1  ->  Sabertooth S1
//                          //   Arduino GND    ->  Sabertooth 0V
//                          //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
//                          //
//                          // If you want to use a pin other than TX->1, see the SoftwareSerial example.

// // Encoder myEnc1((uint8_t)18, (uint8_t)19);
// Encoder myEnc2((uint8_t)5, (uint8_t)6);

// void setup()
// {
//   // Serial.begin(9600);
//   Serial1.begin(19200);
//   // SabertoothTXPinSerial.begin(19200); // This is the baud rate you chose with the DIP switches.
// }

// void loop()
// {

//   // int power;
  
//   // Ramp motor 1 and motor 2 from -127 to 127 (full reverse to full forward),
//   // waiting 20 ms (1/50th of a second) per value.
//   // for (power = -127; power <= 127; power ++)
//   // {
//   //   ST.motor(1, power);
//   //   ST.motor(2, power);
//   //   delay(20);
//   // }
  
//   // // Now go back the way we came.
//   // for (power = 127; power >= -127; power --)
//   // {
//   //   ST.motor(1, power);
//   //   ST.motor(2, power);
//   //   delay(20);
//   // }


//   // long newPosition1 = myEnc1.readAndReset();
//   long newPosition2 = myEnc2.readAndReset();

  
//   String outstr = "test: ";
//   // outstr += newPosition1;
//   // outstr += " / ";
//   outstr += newPosition2;

//   Serial.println(outstr);

//   // int speed = 50;
//   // // Serial.println("test"); 
//   ST.motor(1, 123);  // Go forward at full power.  
//   ST.motor(2, 123);  // Go forward at full power.  
  
//   delay(2000);       // Wait 2 seconds.
//   // ST.motor(1, -20);  // Go forward at full power.
//   // delay(2000);       // Wait 2 seconds.
//   // ST.motor(2, speed);  // Go forward at full power.
//   // delay(2000);       // Wait 2 seconds.
//   // ST.motor(1, 0);    // Stop.
//   // ST.motor(2, 0);    // Stop.
//   // delay(2000);       // Wait 2 seconds.
//   // ST.motor(1, -speed); // Reverse at full power.
//   // ST.motor(2, -speed); // Reverse at full power.
//   // delay(2000);       // Wait 2 seconds.
//   // ST.motor(1, 0);    // Stop.
//   // ST.motor(2, 0);    // Stop.
  
//   // delay(2000);
// }