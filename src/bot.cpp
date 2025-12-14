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

  set_heading();

  pinMode(buttonPIN, INPUT);
}

void Bot::update() {
  getSensorData();

  constexpr float RADIUS = 30.0f;
  constexpr float SPEED = 20.0f;
  constexpr float ANGLE_STEP = 0.015f;

  static float theta = 0.0f;

  const float target_x = RADIUS * cos(theta);
  const float target_y = RADIUS * sin(theta);

  float vx_f = target_x - _x_pos;
  float vy_f = target_y - _y_pos;

  if (const float len = sqrt(vx_f * vx_f + vy_f * vy_f); len > 0.001f) {
    vx_f = (vx_f / len) * SPEED;
    vy_f = (vy_f / len) * SPEED;
  }

  theta += ANGLE_STEP;
  if (theta > TWO_PI) {
    theta -= TWO_PI;
  }

  const int vx = static_cast<int>(roundf(vx_f));
  const int vy = static_cast<int>(roundf(vy_f));

  const int rot = 0 - readCompass() / 4;

  pushData(_ena, false, -vx, -vy, rot, 0);
}

void Bot::getSensorData() {
  // US Positon
  constexpr uint8_t numBytes = 4;
  I2C::requestData(usCircuit, numBytes);
  float local_x = 0;
  float local_y = 0;

  if (I2C::available() >= numBytes) {
    const uint8_t xLow  = I2C::read();
    const uint8_t xHigh = I2C::read();
    const uint8_t yLow  = I2C::read();
    const uint8_t yHigh = I2C::read();

    const uint16_t x_u = (static_cast<uint16_t>(xHigh) << 8) | static_cast<uint16_t>(xLow);
    const uint16_t y_u = (static_cast<uint16_t>(yHigh) << 8) | static_cast<uint16_t>(yLow);

    const int16_t x = static_cast<int16_t>(x_u);
    const int16_t y = static_cast<int16_t>(y_u);

    constexpr float scale = 100.0f;
    local_x = static_cast<float>(x) / scale;
    local_y = static_cast<float>(y) / scale;
  }

  _head = readCompass();

  readButton();

  localToWorld(local_x, local_y, _head, _x_pos, _y_pos);

}

int Bot::readCompass() {
  constexpr uint8_t Angle = 1;
  I2C::transmit(imuAddress, &Angle, 1);
  I2C::requestData(imuAddress, 3);
  while (I2C::available() < 3)
    ;
  unsigned char angle8 = I2C::read();  // Read back the 5 bytes
  const unsigned char high_byte = I2C::read();
  const unsigned char low_byte = I2C::read();
  unsigned int angle16 = high_byte;  // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  const unsigned int precalc = angle16 / 10;

  return ((((precalc - _head_calib) + 180 + 360) % 360) - 180);
}

void Bot::set_heading() {
  _head_calib = readCompass();
}

void Bot::localToWorld(const float lx, const float ly,const float heading_deg, float &gx, float &gy) {
  const float theta = heading_deg * (PI / 180.0f);
  gx = cosf(theta) * lx - sinf(theta) * ly;
  gy = sinf(theta) * lx + cosf(theta) * ly;
}

void Bot::readButton() {
  if (digitalRead(buttonPIN) == HIGH) {
    _ena = !_ena;
  }
  while (digitalRead(buttonPIN) == HIGH)
    ;
}

void Bot::pushData(const bool enable, const bool kick, int vx, int vy, const uint8_t rotation, const uint8_t dribbler) {
  uint8_t data[5];

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

  data[1] = vx_byte;  // vx (0..255)
  data[2] = vy_byte;  // vy (0..255)
  data[3] = rotation; // rotation (0..255)
  data[4] = dribbler; // dribbler

  I2C::transmit(motorDriverMB, data, 5);
}

