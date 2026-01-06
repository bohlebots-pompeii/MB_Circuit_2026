//
// Created by julius on 05.01.2026.
//

#include "../include/Sensors.h"
#include <Arduino.h>
#include <config.h>
#include <iostream>
#include <Wire.h>

Sensors::Sensors(const std::shared_ptr<CM5> &cm5) {
  // Check if the shared pointer for CM5 is valid
  if (cm5 == nullptr) {
    std::cout << "cm5 not existent" << std::endl;
  }

  _cm5 = cm5;

  // Initialize the button pin as input
  pinMode(buttonPIN, INPUT);
}

void Sensors::update() {
  updateLineSensor();
  updateUS();
  updateButton();
}

void Sensors::updateLineSensor() {
  constexpr uint8_t len = 4;
  // Request 4 bytes from the line sensor via I2C
  Wire.requestFrom(lineSensorAddress, len);
  if (Wire.available() >= len) {
    const uint8_t progressLow  = Wire.read();
    const uint8_t progressHigh = Wire.read();
    const uint8_t lineRotLow  = Wire.read();
    const uint8_t lineRotHigh = Wire.read();

    // Reconstruct 16-bit unsigned integers from high and low bytes
    const uint16_t lineRot_u = (static_cast<uint16_t>(lineRotHigh) << 8) | static_cast<uint16_t>(lineRotLow);
    const uint16_t progress_u = (static_cast<uint16_t>(progressHigh) << 8) | static_cast<uint16_t>(progressLow);

    // convert to 16-bit signed integers
    line_rot = static_cast<int16_t>(lineRot_u);
    progress = static_cast<int16_t>(progress_u);

    // Adjustment logic for rotation based on progress
    if (progress >= 16) {
      line_rot += 180;
    }

    // Wrap around rotation if it exceeds 360 degrees
    if (line_rot > 360) {
      line_rot -= 360;
    }
  } else {
    // Reset values if communication fails preventing stuck values
    line_rot = -1;
    progress = -1;
  }
}

void Sensors::updateUS() {
  constexpr uint8_t numBytes = 4;
  // Request 4 bytes containing coordinate data from US sensor
  Wire.requestFrom(usAddress, numBytes);

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
    const float local_x = static_cast<float>(x) / scale;
    const float local_y = static_cast<float>(y) / scale;

    float g_x;
    float g_y;
    const float heading = _cm5->getHeading();

    localToWorld(local_x, local_y, heading, g_x, g_y);

    position.setX(g_x);
    position.setY(g_y);
  }
}

void Sensors::updateButton() {
  const bool currentButtonState = digitalRead(buttonPIN);
  if (currentButtonState == HIGH && !lastButtonState) {
    ena = !ena;
  }
  lastButtonState = currentButtonState;
}

void Sensors::localToWorld(const float lx, const float ly,const float heading_deg, float &gx, float &gy) {
  // Convert heading to radians for trigonometric functions
  const float theta = heading_deg * (PI / 180.0f);
  // Apply rotation matrix to convert local point (lx, ly) to global point (gx, gy)
  gx = cosf(theta) * lx - sinf(theta) * ly;
  gy = sinf(theta) * lx + cosf(theta) * ly;
}