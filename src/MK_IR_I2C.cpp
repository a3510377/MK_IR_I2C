#include "MK_IR_I2C.h"

MK_IR_I2C::MK_IR_I2C(uint8_t addr, TwoWire *wire) : _addr(addr), _wire(wire) {}

bool MK_IR_I2C::begin() {
  return isConnected();
}

bool MK_IR_I2C::isConnected() {
  _wire->beginTransmission(_addr);

  return _wire->endTransmission() == 0;
}

bool MK_IR_I2C::_writeRegister(uint8_t reg, const uint8_t *data, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(data, len);
  return _wire->endTransmission() == 0;
}

bool MK_IR_I2C::_readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) return false;
  if (_wire->requestFrom(_addr, len) != len) return false;
  return _wire->readBytes(data, len) == len;
}
