//
// Created by julius on 05.01.2026.
//

#include "../include/Sensors.h"
#include <Arduino.h>
#include <config/config.h>

#include <Wire.h>
#include <elapsedMillis.h>

elapsedMillis ledBlinkTimer;

Sensors::Sensors(const std::shared_ptr<CM5>& cm5) {
  // Check if the shared pointer for CM5 is valid
  if (cm5 == nullptr) {
    Serial.println("[Sensors] ERROR: cm5 is null");
  }

  _cm5 = cm5;

  // Initialize the button pin as input
  pinMode(PINS::SINGLE_BUTTON_PIN, INPUT);
  // Light gate
  pinMode(PINS::LIGHT_GATE_PIN, INPUT);
  // comms module
  pinMode(PINS::COMMS_MODULE_PIN, INPUT);
}

void Sensors::update() {
  updateLineSensor();
  // updateUS();
  updateButtons();
  lightGateAvg.addValue(analogRead(PINS::LIGHT_GATE_PIN));
}

void Sensors::updateLineSensor() {
  static int16_t lastLineRot = -1;
  static int16_t lastLineProgress = -1;
  constexpr uint8_t len = 4;
  // Request 4 bytes from the line sensor via I2C
  Wire.requestFrom(I2C_ADDRESSES::LINE_ADDR, len);
  if (Wire.available() >= len) {
    const uint8_t progressLow = Wire.read();
    const uint8_t progressHigh = Wire.read();
    const uint8_t lineRotLow = Wire.read();
    const uint8_t lineRotHigh = Wire.read();

    // Reconstruct 16-bit unsigned integers from high and low bytes
    const uint16_t lineRot_u = (static_cast<uint16_t>(lineRotHigh) << 8) | static_cast<uint16_t>(lineRotLow);
    const uint16_t progress_u = (static_cast<uint16_t>(progressHigh) << 8) | static_cast<uint16_t>(progressLow);

    // convert to 16-bit signed integers
    line_rot = static_cast<int16_t>(lineRot_u);
    progress = static_cast<int16_t>(progress_u);

    if (line_rot != -1 && progress != -1) {
      // Adjust line rotation based on progress and approach direction
      static bool coming_from_front = false;
      if (lastLineProgress != 16) {
        coming_from_front = (lastLineRot >= 0 && (lastLineRot < 90 || lastLineRot >= 270));
      }

      if (coming_from_front) {
        if (progress >= 16) {
          line_rot += 180;
        }
      }
      else {
        if (progress > 16) {
          line_rot += 180;
        }
      }

      if (line_rot >= 360) {
        line_rot -= 360;
      }
    }

    lastLineRot = line_rot;
    lastLineProgress = progress;
  }
  else {
    // Reset values if communication fails preventing stuck values
    line_rot = -1;
    progress = -1;
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
  // cycles Sensors::GREEN(1) through Sensors::WHITE(7)
  static uint8_t LEDColorCounter = 1;
  static boolean ledOn = true;
  if (ledBlinkTimer < 250) {
    if (ledOn) {
      for (int i = 0; i < 8; ++i) {
        setLED(i, 1, LEDColorCounter);
        setLED(i, 2, LEDColorCounter);
      }
      return;
    }
    for (int i = 0; i < 8; ++i) {
      setLED(i, 1, OFF);
      setLED(i, 2, OFF);
    }
  }
  else {
    ledBlinkTimer = 0;
    ledOn = !ledOn;
  }
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
  }
  else if (nr == 2) {
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
