//
// created by Julius 03.12.2025.
//

#ifndef BOHLEBOTS_2026_I2C_H
#define BOHLEBOTS_2026_I2C_H
#include <Arduino.h>

class I2C {
public:
  static void init();

  static void requestData(int addr, int numBytes);

  static int available();

  static uint8_t read();
  
  static void transmit(uint8_t address, const uint8_t* data, size_t length);
};
#endif