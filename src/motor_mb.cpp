//
// Created by julius on 05.01.2026.
//

#include "motor_mb.h"
#include <Arduino.h>
#include <Wire.h>
#include <config.h>

void pushData(const bool enable, const bool kick, int vx, int vy, int rot, int dribbler) {
  MotorCmd cmd{};

  vx *= -1;
  vy *= -1;
  rot *= -1;

  vx = constrain(vx, -100, 100);
  vy = constrain(vy, -100, 100);
  rot = constrain(rot, -100, 100);
  dribbler = constrain(dribbler, -100, 100);

  cmd.flags = 0;
  if (enable) cmd.flags |= 0x01;
  if (kick)   cmd.flags |= 0x02;

  cmd.vx   = static_cast<int8_t>(vx);
  cmd.vy   = static_cast<int8_t>(vy);
  cmd.rot  = static_cast<int8_t>(rot);
  cmd.drib = static_cast<int8_t>(dribbler);

  Wire.beginTransmission(motorMBAddress);
  Wire.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
  Wire.endTransmission();
}