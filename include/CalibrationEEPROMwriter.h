#ifndef __CALIBRATION_EEPROM_WRITER__
#define __CALIBRATION_EEPROM_WRITER__

#include <Arduino.h>
#include <EEPROM.h>

/*
  Modified version of AdaFruit's EEPROM code from the 
  Adafruit_Sensor_Calibration package.
  For accelerometer, we are using an advanced calibration method that
  results in 12 floats to be stored, compared to the 3 floats 
  that the original Adafruit calibration library used.
  So these modifications allow us store 9*4=36 extra bytes.
*/

// We will be writing 104 bytes:
// Gyroscope: 3 offset values (float)
// Magnetometer: 3 offset values (float) and 9 scaling values (float)
// Accelerometer: 3 offset values (float), 6 scaling values (float), and one magnetic magnitude value (float)
// Total: (9+9+6+1)*4 = 100 bytes.
// The other 4 bytes are for header and CRC checksum.
#define EEPROM_CAL_SIZE 104

typedef union _gyro_offset
{
    float offset[3] = {0.0, 0.0, 0.0};
    struct _offsets
    {
        float x;
        float y;
        float z;			
    }Offsets;
}GyroOffset;

typedef union _mag_offset
{
    float offset[3] = {0.0, 0.0, 0.0};
    struct _offsets
    {
        float x;
        float y;
        float z;			
    }Offsets;
}MagOffset;

typedef union _mag_scaling
{
    float scaling[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    struct _scaling
    {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    }Scaling;
}MagScaling;

typedef union _accel_offset
{
    float offset[3]= {0.0, 0.0, 0.0};
    struct _offsets
    {
        float x;
        float y;
        float z;			
    }Offsets;
}AccelOffset;

typedef union _accel_scaling
{
    float scaling[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    struct _scaling
    {
        float xx;
        float xy;
        float xz;
        float yx;
        float yy;
        float yz;
        float zx;
        float zy;
        float zz;
    }Scaling;
}AccelScaling;

typedef struct _sensor_calibration{
    GyroOffset gyro_offset;
    AccelOffset accel_offset;
    AccelScaling accel_scaling;
    MagOffset mag_offset;
    MagScaling mag_scaling;
    float mag_field = 0.0;
    
}SensorCalibration;

class CalibrationEEPROMwriter {
public:
  bool begin(uint8_t eeprom_addr = 60);

  bool saveCalibration(const SensorCalibration& calib_params);
  bool loadCalibration(SensorCalibration* calib_params);  

private:
  uint16_t ee_addr = 0;
  uint16_t crc16_update(uint16_t crc, uint8_t a);
};

#endif