//
// Created by julius on 05.01.2026.
//

#include "../include/Sensors.h"
#include <Arduino.h>
#include <config.h>
#include <iostream>
#include <Wire.h>

Sensors::Sensors(const std::shared_ptr<CM5> &cm5) {
  if (cm5 == nullptr) {
    std::cout << "cm5 not existent" << std::endl;
  }

  _cm5 = cm5;

  pinMode(buttonPIN, INPUT);
}

void Sensors::update() {
  updateLineSensor();
  updateUS();
  updateButton();
}

void Sensors::updateLineSensor() {
  constexpr uint8_t len = 4;
  Wire.requestFrom(lineSensorAddress, len);
  if (Wire.available() >= len) {
    const uint8_t progressLow  = Wire.read();
    const uint8_t progressHigh = Wire.read();
    const uint8_t lineRotLow  = Wire.read();
    const uint8_t lineRotHigh = Wire.read();

    const uint16_t lineRot_u = (static_cast<uint16_t>(lineRotHigh) << 8) | static_cast<uint16_t>(lineRotLow);
    const uint16_t progress_u = (static_cast<uint16_t>(progressHigh) << 8) | static_cast<uint16_t>(progressLow);

    line_rot = static_cast<int16_t>(lineRot_u);
    progress = static_cast<int16_t>(progress_u);

    if (progress >= 16) {
      line_rot += 180;
    }

    if (line_rot > 360) {
      line_rot -= 360;
    }
  }
}

void Sensors::updateUS() {
  constexpr uint8_t numBytes = 4;
  Wire.requestFrom(usAddress, numBytes);
  float local_x = 0;
  float local_y = 0;

  if (Wire.available() >= numBytes) {
    const uint8_t xLow  = Wire.read();
    const uint8_t xHigh = Wire.read();
    const uint8_t yLow  = Wire.read();
    const uint8_t yHigh = Wire.read();

    const uint16_t x_u = (static_cast<uint16_t>(xHigh) << 8) | static_cast<uint16_t>(xLow);
    const uint16_t y_u = (static_cast<uint16_t>(yHigh) << 8) | static_cast<uint16_t>(yLow);

    const int16_t x = static_cast<int16_t>(x_u);
    const int16_t y = static_cast<int16_t>(y_u);

    constexpr float scale = 100.0f;
    local_x = static_cast<float>(x) / scale;
    local_y = static_cast<float>(y) / scale;
  }

  float g_x;
  float g_y;
  const float heading = _cm5->getHeading();

  localToWorld(local_x, local_y, heading, g_x, g_y);

  position.setX(g_x);
  position.setY(g_y);
}

void Sensors::updateButton() {
  if (digitalRead(buttonPIN) == HIGH) {
    ena = !ena;
  }
  while (digitalRead(buttonPIN) == HIGH)
    ;
}

void Sensors::localToWorld(const float lx, const float ly,const float heading_deg, float &gx, float &gy) {
  const float theta = heading_deg * (PI / 180.0f);
  gx = cosf(theta) * lx - sinf(theta) * ly;
  gy = sinf(theta) * lx + cosf(theta) * ly;
}