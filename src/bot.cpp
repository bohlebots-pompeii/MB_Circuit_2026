//
// Created by julius on 11.11.2025.
//

#include <bot.h>
#include <config.h>
#include <comms/i2c.h>
#include <Arduino.h>
#include <Vector2.hpp>

void Bot::init() {
  I2C::init();
}

void Bot::update() {
  getSensorData();

  /*
  static float angle = 0.0f;
  angle += 0.075f;

  const float vx_float = 30.0f * cos(angle);
  const float vy_float = 30.0f * sin(angle);

  const int8_t vx = static_cast<int8_t>(map(static_cast<int>(vx_float), -100, 100, 0, 255));
  const int8_t vy = static_cast<int8_t>(map(static_cast<int>(vy_float), -100, 100, 0, 255));
  */
}

void Bot::getSensorData() {
  constexpr uint8_t numBytes = 4;
  I2C::requestData(usCircuit, numBytes);

  if (I2C::available() >= numBytes) {
    const uint8_t xLow  = I2C::read();
    const uint8_t xHigh = I2C::read();
    const uint8_t yLow  = I2C::read();
    const uint8_t yHigh = I2C::read();

    const uint16_t x_u = (static_cast<uint16_t>(xHigh) << 8) | static_cast<uint16_t>(xLow);
    const uint16_t y_u = (static_cast<uint16_t>(yHigh) << 8) | static_cast<uint16_t>(yLow);

    const int16_t x = static_cast<int16_t>(x_u);
    const int16_t y = static_cast<int16_t>(y_u);

    constexpr float scale = 100.0f; // cm
    const float x_pos = static_cast<float>(x) / scale;
    const float y_pos = static_cast<float>(y) / scale;
    Serial.print("pos: ");
    Serial.print(x_pos);
    Serial.print(", ");
    Serial.println(y_pos);

    const float vx_f = (y_pos / 8.0f) * 3.0f;
    const float vy_f = (x_pos / 8.0f) * 3.0f;

    const int vx = static_cast<int>(roundf(vx_f));
    const int vy = static_cast<int>(roundf(vy_f));

    pushData(true, false, vx, vy, 0, 0);
  }
}

void Bot::pushData(const bool enable, const bool kick, int vx, int vy, const uint8_t rotation, const uint8_t dribbler) {
  uint8_t data[5];

  vx *= -1;
  vy *= -1;

  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);

  const long mapped_vx = map(vx, -100, 100, 0, 255);
  const long mapped_vy = map(vy, -100, 100, 0, 255);

  const uint8_t vx_byte = static_cast<uint8_t>(constrain(mapped_vx, 0L, 255L));
  const uint8_t vy_byte = static_cast<uint8_t>(constrain(mapped_vy, 0L, 255L));

  // Byte 1 -> ena, kick
  data[0] = 0;
  if (enable) data[0] |= 0x01;  // Bit 0
  if (kick)   data[0] |= 0x02;  // Bit 1

  data[1] = vx_byte;       // vx (0..255)
  data[2] = vy_byte;       // vy (0..255)
  data[3] = rotation; // rotation (0..255)
  data[4] = dribbler; // dribbler

  I2C::transmit(motorDriverMB, data, 5);
}

