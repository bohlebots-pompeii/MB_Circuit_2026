//
// Created by julius on 05.01.2026.
//

#include "motor_mb.h"
#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include <Vector2.hpp>
#include <numbers>

double rotDelta = 0.0;

void setRotDelta(const double delta) {
  rotDelta = delta;
  rotDelta = rotDelta / 180.0f * std::numbers::pi;
}

void pushData(const bool enable, const bool kick, int vx, int vy, int rot, int dribbler) {
  MotorCmd cmd{};

  rot *= -1;

  auto global_drive = Vector2(vx, vy);
  global_drive.rotate(-rotDelta * 2.0f); // compensate for rotation delta

  const int vx_rot = constrain(global_drive.getX(), -40, 40);
  const int vy_rot = constrain(global_drive.getY(), -40, 40);
  rot = constrain(rot, -50, 50);
  dribbler = constrain(dribbler, -100, 100);

  cmd.flags = 0;
  if (enable) cmd.flags |= 0x01;
  if (kick)   cmd.flags |= 0x02;

  cmd.vx   = static_cast<int8_t>(-vy_rot); // swapped for normal math
  cmd.vy   = static_cast<int8_t>(-vx_rot);
  cmd.rot  = static_cast<int8_t>(rot);
  cmd.drib = static_cast<int8_t>(dribbler);

  Wire.beginTransmission(motorMBAddress);
  Wire.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
  Wire.endTransmission();
}