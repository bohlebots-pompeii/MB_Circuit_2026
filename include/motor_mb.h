//
// Created by julius on 05.01.2026.
//

#pragma once

#include <Arduino.h>

struct __attribute__((packed)) MotorCmd {
  uint8_t flags;   // bit0 = ena, bit1 = kick
  int8_t  vx;      // -100 .. 100
  int8_t  vy;      // -100 .. 100
  int8_t  rot;     // -100 .. 100
  int8_t  drib;    // -100 .. 100
};

void pushData(bool enable, bool kick, int vx, int vy, int rot, int dribbler, bool useRotDelta, double rotDeltaRad = 0.0);