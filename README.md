# FastAHRS

Part of my self balancing robot project.

Robust 400Hz Atitude and Heading (AHRS) estimation for the AdaFruit 
"Precision NXP 9-DOF breakout board - FXOS8700 + FXAS21002".
This library solves a bunch of issues that come with the default AdaFruit
libraries, and uses the latest updates of the MadgWick filter that few people 
seem to be aware of.

This implementation runs at 400Hz on a Teensy 4.0 microcontroller.
It contains:
- Sensor fusion (accelerometer, gyroscope, magnetometer) for attitude estimation
- Outlier removal for magnetometer values to deal with environmental factors
- Outlier removal for accelerometer values to deal with fast motion
- Advanced accelerometer calibration (12 parameters; 9 for scaling, 3 for offsets)
- Support to drive pin output for indicator leds showing anomaly and initialization status

This repo also contains a Python notebook to perform accelerometer calibration, by posing 
it as an optimization problem. The advantage of this approach is that you don't need to hold
the IMU in any specific orientation, making calibration much easier and accurate than with 
most other techniques.

To use this, you must calibrate the sensors, and write the calibration parameters
to EEPROM. Check the calibrate_accel, calibrate_gyro and calibrate_magnetometer samples
in the 'examples' folder to obtain the calibration parameters.
You can use the 'CalibrationEEPROM_read_write.cpp' file to write the values to EEPROM.

The underlying attitude estimation uses the Madgwick complementary filter.
Note: This is not based on the older gradient based Madgwick filter that lots of people 
still use! Instead, it's based on the latest version of his attitude estimator that 
Madgwick himself recommends: https://github.com/xioTechnologies/Fusion

In Sebastian Madgwick's PhD thesis, titled "AHRS algorithms and calibration solutions to
facilitate new applications using low-cost MEMS", he first discusses the gradient 
based filter that most people know about. At the end of his thesis, he then discusses his
'revised algorithm', which offers several improvements upon the original algorithm,
is much more efficient and robust against bad sensor readings from typical low-cost sensors,
and is similar to the Mahoney algorithm.

Note: The AdaFruit default AHRS libraries only run at 100Hz by default, and do
not support full 12DOF accelerometer calibration.  Also, the AdaFruit sensor 
abstractions are  not very efficient when used for the precision NXP breakout-board, 
because the abstractions force you to query accelerometer and magnetometer in separate 
calls, while they can and should be polled in a single instruction. 
This library solves all of that.

Finally, the default AdaFruit implementation configures the gyroscope for a 250dps 
sensitivity. However, this is not enough if you want to cope with fast motion 
(e.g. shaking the sensor by hand). In this implementation, we configure the gyroscope 
for a 1000dps sensitivity instead.

To correctly run at 400 HZ, you will have to make the following changes to the
Adafruit sensor drivers that are used:
Adafruit_FXOS8700.cpp: Line 184 (use 0x15 for 100Hz, 0x05 for 400Hz)
Adafruit_FXAS21002C.cpp: Line 203 (use 0x0E for 100Hz, 0x06 for 400Hz)