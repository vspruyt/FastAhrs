#include "CalibrationEEPROMwriter.h"

/*
  Modified version of AdaFruit's EEPROM code from the 
  Adafruit_Sensor_Calibration package.
  For accelerometer, we are using an advanced calibration method that
  results in 12 floats to be stored, compared to the 3 floats 
  that the original Adafruit calibration library used.
  So these modifications allow us store 9*4=36 extra bytes.
*/
bool CalibrationEEPROMwriter::begin(uint8_t eeprom_addr) {
  ee_addr = eeprom_addr;

  return true;
}

/**************************************************************************/
/*!
    @brief CRC16 calculation helper with 0xA001 seed
    @param crc Last byte's CRC value
    @param a The new byte to append-compute
    @returns New 16 bit CRC
*/
/**************************************************************************/
uint16_t CalibrationEEPROMwriter::crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a;
  for (i = 0; i < 8; i++) {
    if (crc & 1) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

bool CalibrationEEPROMwriter::saveCalibration(const SensorCalibration& calib_params) {
  Serial.println("Save Cal");

  uint8_t buf[EEPROM_CAL_SIZE];
  memset(buf, 0, EEPROM_CAL_SIZE);
  buf[0] = 0x75;
  buf[1] = 0x54;

  float offsets[25];
  memcpy(offsets, calib_params.accel_offset.offset, 12);       // 3 x 4-byte floats
  memcpy(offsets + 3, calib_params.accel_scaling.scaling, 36); // 9 x 4-byte floats
  memcpy(offsets + 12, calib_params.gyro_offset.offset, 12);   // 3 x 4-byte floats
  memcpy(offsets + 15, calib_params.mag_offset.offset, 12);    // 3 x 4-byte floats

  offsets[18] = calib_params.mag_field;

  offsets[19] = calib_params.mag_scaling.scaling[0];
  offsets[20] = calib_params.mag_scaling.scaling[4];
  offsets[21] = calib_params.mag_scaling.scaling[8];
  offsets[22] = calib_params.mag_scaling.scaling[1];
  offsets[23] = calib_params.mag_scaling.scaling[2];
  offsets[24] = calib_params.mag_scaling.scaling[5];

  memcpy(buf + 2, offsets, 25 * 4);

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < EEPROM_CAL_SIZE - 2; i++) {
    crc = crc16_update(crc, buf[i]);
  }
  Serial.print("CRC: ");
  Serial.println(crc, HEX);
  buf[EEPROM_CAL_SIZE - 2] = crc & 0xFF;
  buf[EEPROM_CAL_SIZE - 1] = crc >> 8;

  for (uint16_t a = 0; a < EEPROM_CAL_SIZE; a++) {
    EEPROM.write(a + ee_addr, buf[a]);
  }

#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
  return true;
}

bool CalibrationEEPROMwriter::loadCalibration(SensorCalibration* calib_params) {
  uint8_t buf[EEPROM_CAL_SIZE];

  uint16_t crc = 0xFFFF;
  for (uint16_t a = 0; a < EEPROM_CAL_SIZE; a++) {
    buf[a] = EEPROM.read(a + ee_addr);
    crc = crc16_update(crc, buf[a]);
  }

  if (crc != 0 || buf[0] != 0x75 || buf[1] != 0x54) {
    Serial.print("CRC: ");
    Serial.println(crc, HEX);
    return false;
  }

  float offsets[25];
  memcpy(offsets, buf + 2, 25 * 4);
  calib_params->accel_offset.offset[0] = offsets[0];
  calib_params->accel_offset.offset[1] = offsets[1];
  calib_params->accel_offset.offset[2] = offsets[2];

  calib_params->accel_scaling.scaling[0] = offsets[3];
  calib_params->accel_scaling.scaling[1] = offsets[4];
  calib_params->accel_scaling.scaling[2] = offsets[5];
  calib_params->accel_scaling.scaling[3] = offsets[6];
  calib_params->accel_scaling.scaling[4] = offsets[7];
  calib_params->accel_scaling.scaling[5] = offsets[8];
  calib_params->accel_scaling.scaling[6] = offsets[9];
  calib_params->accel_scaling.scaling[7] = offsets[10];
  calib_params->accel_scaling.scaling[8] = offsets[11];

  calib_params->gyro_offset.offset[0] = offsets[12];
  calib_params->gyro_offset.offset[1] = offsets[13];
  calib_params->gyro_offset.offset[2] = offsets[14];

  calib_params->mag_offset.offset[0] = offsets[15];
  calib_params->mag_offset.offset[1] = offsets[16];
  calib_params->mag_offset.offset[2] = offsets[17];

  calib_params->mag_field = offsets[18];

  calib_params->mag_scaling.scaling[0] = offsets[19];
  calib_params->mag_scaling.scaling[1] = offsets[22];
  calib_params->mag_scaling.scaling[2] = offsets[23];
  calib_params->mag_scaling.scaling[3] = offsets[22];
  calib_params->mag_scaling.scaling[4] = offsets[20];
  calib_params->mag_scaling.scaling[5] = offsets[24];
  calib_params->mag_scaling.scaling[6] = offsets[23];
  calib_params->mag_scaling.scaling[7] = offsets[24];
  calib_params->mag_scaling.scaling[8] = offsets[21];

  return true;
}