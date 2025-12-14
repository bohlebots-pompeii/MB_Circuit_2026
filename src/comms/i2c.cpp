//
// created by Julius on 3.12.2025.
//

#include "comms/i2c.h"
#include <Wire.h>

void I2C::init() {
  Wire.begin();
}

void I2C::requestData(const int addr, const int numBytes) {
  Wire.requestFrom(addr, numBytes);
}

int I2C::available() {
  return Wire.available();
}

uint8_t I2C::read() {
  return Wire.read();
}

void I2C::transmit(const uint8_t address, const uint8_t* data,const size_t length) {
  Wire.beginTransmission(address);
  Wire.write(data, length);
  Wire.endTransmission();
}
