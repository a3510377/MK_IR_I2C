#pragma once

#include <Wire.h>
#include <inttypes.h>

#define MK_IR_I2C_ADDR_DEFAULT 0x50

enum {
  MK_IR_I2C_LED_ADDR                 = 0x01,
  MK_IR_I2C_LED_MODE_MASK_ADDR       = 0x02,
  MK_IR_I2C_READ_THRESHOLD_ONE_ADDR  = 0x13,
  MK_IR_I2C_WRITE_THRESHOLD_ONE_ADDR = 0x14,
  MK_IR_I2C_THRESHOLD_ALL_ADDR       = 0x15,
  MK_IR_I2C_THRESHOLD_FLAG_ADDR      = 0x16,
  MK_IR_I2C_ADC_DATA_ADDR            = 0x29,
  MK_IR_I2C_ADC_DATA00_ADDR          = 0x30,
  MK_IR_I2C_ADC_DATA01_ADDR          = 0x31,
  MK_IR_I2C_ADC_DATA02_ADDR          = 0x32,
  MK_IR_I2C_ADC_DATA03_ADDR          = 0x33,
  MK_IR_I2C_ADC_DATA04_ADDR          = 0x34,
  MK_IR_I2C_ADC_DATA05_ADDR          = 0x35,
  MK_IR_I2C_ADC_DATA06_ADDR          = 0x36,
  MK_IR_I2C_ADC_DATA07_ADDR          = 0x37,
  MK_IR_I2C_ADC_DATA08_ADDR          = 0x38,
  MK_IR_I2C_ADC_DATA09_ADDR          = 0x39,
  MK_IR_I2C_ADC_DATA10_ADDR          = 0x3A,
  MK_IR_I2C_ADC_DATA11_ADDR          = 0x3B,
  MK_IR_I2C_ADC_DATA12_ADDR          = 0x3C,
  MK_IR_I2C_ADC_DATA13_ADDR          = 0x3D,
  MK_IR_I2C_ADC_DATA14_ADDR          = 0x3E,
  MK_IR_I2C_ADC_DATA15_ADDR          = 0x3F,
  MK_IR_I2C_VERSION_ADDR             = 0xFE,
  MK_IR_I2C_LAST_ERROR_ADDR          = 0xFF,
};

class MK_IR_I2C {
 public:
  MK_IR_I2C(uint8_t addr = MK_IR_I2C_ADDR_DEFAULT, TwoWire *wire = &Wire);

  bool begin();
  bool isConnected();

 private:
  uint8_t _addr;
  TwoWire *_wire;

  bool _writeRegister(uint8_t reg, const uint8_t *data, uint8_t len);
  bool _readRegister(uint8_t reg, uint8_t *data, uint8_t len);

  friend class MK_IR_I2C_LED;
};

// class MK_IR_I2C_LED {
//  public:
//   MK_IR_I2C_LED(MK_IR_I2C *ir);

//   uint16_t operator[](uint8_t index);

//   MK_IR_I2C_LED &operator()(uint8_t index, uint16_t value);

//   bool update();
//   bool autoUpdate();
//   bool autoUpdate(bool enable);

//  private:
//   MK_IR_I2C *_ir;

//   bool _auto_update;
//   uint16_t _old_data;
// };
