//
// Created by julius on 05.01.2026.
//

#pragma once

#include <util/Vector2.hpp>
#include <util/MovingAverage.h>
#include <memory>
#include <comms/CM5.h>
#include <Arduino.h>
#include "config/config.h"

class Sensors {
public:
  explicit Sensors(const std::shared_ptr<CM5>& cm5);

  void update();

  void setEna(bool state);
  void allLEDsOff();
  void haltLEDs();

  [[nodiscard]] bool getButtonState(int device, int nr) const;
  void setLED(int device, int nr, int color);

  // getters
  [[nodiscard]] int16_t getLineRot() const { return line_rot; }

  [[nodiscard]] bool getLineSeen() const { return progress != -1 && line_rot != -1; }

  [[nodiscard]] bool getHasBall() const { return lightGateAvg.getAverage() > 2800; }
  [[nodiscard]] int getBallLightGate() const { return lightGateAvg.getAverage(); }

  [[nodiscard]] int16_t getProgress() const { return progress; }

  [[nodiscard]] bool getEna() const { return ena; }

  [[nodiscard]] static bool getForceHalt() {
    static bool stableState = true; // Default to TRUE (halted) safely
    static uint32_t lastChangeMs = 0;
    static bool lastRaw = true;

    const bool raw = (digitalRead(PINS::COMMS_MODULE_PIN) == LOW);

    if (raw != lastRaw) {
      lastChangeMs = millis();
      lastRaw = raw;
    }

    // 50ms debounce to prevent voltage dips (when motors enable) from causing instant stops,
    // and to filter out mechanical bouncing from the comms receiver.
    if (millis() - lastChangeMs > 50) {
      stableState = raw;
    }

    return stableState;
  }

  enum COLOR {
    // for the LED colors
    OFF = 0,
    GREEN = 1,
    RED = 2,
    YELLOW = 3,
    BLUE = 4,
    CYAN = 5,
    MAGENTA = 6,
    WHITE = 7
  };

private:
  std::array<bool, 8> portEna = {true, false, false, false, false, false, false, false};

  std::array<int, 8> buttonLedID = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
  std::array<bool, 8> button1Array = {false, false, false, false, false, false, false, false};
  std::array<bool, 8> button2Array = {false, false, false, false, false, false, false, false};

  std::array<int, 8> led1Array = {0, 0, 0, 0, 0, 0, 0, 0};
  std::array<int, 8> led2Array = {0, 0, 0, 0, 0, 0, 0, 0};

  bool ena = false;

  int16_t line_rot = -1;
  int16_t progress = -1;

  MovingAverage<int, 50> lightGateAvg;

  std::shared_ptr<CM5> _cm5;


  void updateLineSensor();
  void updateButtons();
};
