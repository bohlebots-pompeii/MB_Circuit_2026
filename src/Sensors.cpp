//
// Created by julius on 05.01.2026.
//

#include "../include/Sensors.h"
#include <Arduino.h>
#include <config/config.h>
#include <iostream>
#include <Wire.h>
#include <elapsedMillis.h>

elapsedMillis ledBlinkTimer;

Sensors::Sensors(const std::shared_ptr<CM5> &cm5) {
  // Check if the shared pointer for CM5 is valid
  if (cm5 == nullptr) {
    std::cout << "cm5 not existent" << std::endl;
  }

  _cm5 = cm5;

  // Initialize the button pin as input
  pinMode(PINS::buttonPIN, INPUT);
  // Light gate
  pinMode(PINS::lightGatePIN, INPUT);
  // comms module
  pinMode(PINS::communicationModulePIN, INPUT);
}

void Sensors::update() {
  updateLineSensor();
  // updateUS();
  updateButtons();
}

void Sensors::updateLineSensor() {
  static int16_t lastLineRot = -1;
  static int16_t lastLineProgress = -1;
  constexpr uint8_t len = 4;
  // Request 4 bytes from the line sensor via I2C
  Wire.requestFrom(I2C_ADDRESSES::lineSensorAddress, len);
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

    // Adjust line rotation based on progress and approach direction
    static bool coming_from_front = false;
    if (lastLineProgress != 16) {
      coming_from_front = (lastLineRot < 90 || lastLineRot >= 270);
    }

    if (coming_from_front) {
      if (progress >= 16) {
        line_rot += 180;
      }
    } else {
      if (progress > 16) {
        line_rot += 180;
      }
    }

    if (line_rot >= 360) {
      line_rot -= 360;
    }

    lastLineRot = line_rot;
    lastLineProgress = progress;
  } else {
    // Reset values if communication fails preventing stuck values
    line_rot = -1;
    progress = -1;
  }
}

void Sensors::updateUS() {
  constexpr uint8_t numBytes = 4;
  // Request 4 bytes containing coordinate data from US sensor
  Wire.requestFrom(I2C_ADDRESSES::usAddress, numBytes);

  if (Wire.available() >= numBytes) {
    const uint8_t xLow  = Wire.read();
    const uint8_t xHigh = Wire.read();
    const uint8_t yLow  = Wire.read();
    const uint8_t yHigh = Wire.read();

    const uint16_t x_u = (static_cast<uint16_t>(xHigh) << 8) | static_cast<uint16_t>(xLow);
    const uint16_t y_u = (static_cast<uint16_t>(yHigh) << 8) | static_cast<uint16_t>(yLow);

    const auto x = static_cast<int16_t>(x_u);
    const auto y = static_cast<int16_t>(y_u);

    constexpr float scale = 100.0f;
    const float local_x = static_cast<float>(x) / scale;
    const float local_y = static_cast<float>(y) / scale;

    //float g_x;
    //float g_y;
    //const float heading = _cm5->getHeading();

    //localToWorld(local_x, local_y, heading, g_x, g_y);

    position.setX(local_x);
    position.setY(local_y);
  }
}


void Sensors::setEna(const bool state) {
  ena = state;
  if (ena) {
    setLED(0, 1, GREEN);
  }
  else {
    setLED(0, 1, RED);
  }
}

void Sensors::allLEDsOff() {
  for (int i = 0; i < 8; ++i) {
    setLED(i, 1, OFF);
    setLED(i, 2, OFF);
  }
}

void Sensors::haltLEDs() {
  static uint8_t LEDColorCounter = 1;
  if (ledBlinkTimer < 250) {
    for (int i = 0; i < 8; ++i) {
      setLED(i, 1, LEDColorCounter);
      setLED(i, 2, LEDColorCounter);
    }
  }
  else {
    ledBlinkTimer = 0;
    LEDColorCounter++;
  }
  if (LEDColorCounter >= 8) {
    LEDColorCounter = 1;
  }
}

void Sensors::localToWorld(const float lx, const float ly,const float heading_deg, float &gx, float &gy) {
  // Convert heading to radians for trigonometric functions
  const auto theta = static_cast<float>(heading_deg * (PI / 180.0f));
  // Apply rotation matrix to convert local point (lx, ly) to global point (gx, gy)
  gx = cosf(theta) * lx - sinf(theta) * ly;
  gy = sinf(theta) * lx + cosf(theta) * ly;
}

bool Sensors::getButtonState(const int device, const int nr) const {
  if (device < 0 || device > 7) {
    return false;
  }
  if (nr == 1) {
    return button1Array[device];
  }
  if (nr == 2) {
    return button2Array[device];
  }
  return false;
}

void Sensors::setLED(const int device, const int nr, int color) {
  if (color < 0 || color > 7 || device < 0 || device > 7) {
    return;
  }
  if (nr == 1) {
    led1Array[device] = color * 2;
  } else if (nr == 2) {
    color *= 16;
    if (color > 63) {
      color += 64;
    }
    led2Array[device] = color;
  }
}

void Sensors::updateButtons() {
  // from BohleBots header - Roland Stiebel
  for (int lauf = 0; lauf < 8; lauf++) {
    if (portEna[lauf]) {
      int ledwert = 255 - led1Array[lauf] - led2Array[lauf];
      Wire.beginTransmission(buttonLedID[lauf]);
      Wire.write(ledwert);

      Wire.endTransmission();

      Wire.requestFrom(buttonLedID[lauf], 1);
      if (Wire.available()) {
        int tread = 255 - Wire.read();
        tread = tread % 128;
        if (tread > 63)
          button2Array[lauf] = true;
        else
          button2Array[lauf] = false;
        tread = tread % 2;
        if (tread > 0)
          button1Array[lauf] = true;
        else
          button1Array[lauf] = false;
      }
    }
  }
}