//
// Created by julius on 05.01.2026.
//

#include "motor_mb.h"
#include <Arduino.h>
#include <Wire.h>
#include <config/config.h>
#include <util/Vector2.hpp>

#include "MotionController.h"

void pushData(const bool enable, const bool kick, const int vx, const int vy, int rot, int dribbler, const bool useRotDelta) {
  MotorCmd cmd{};

  rot *= -1;

  auto global_drive = Vector2(vx, vy);
  if (useRotDelta) {
    const double rotDeltaRad = MotionController::getInstance() ? MotionController::getInstance()->getRotDeltaRad() : 0.0;
    global_drive.rotate(-rotDeltaRad * 2.0f);
  }

  const int vx_rot = constrain(global_drive.getX(), -70, 70);
  const int vy_rot = constrain(global_drive.getY(), -70, 70);
  rot = constrain(rot, -50, 50);
  dribbler = constrain(dribbler, -100, 100);

  cmd.flags = 0;
  if (enable) cmd.flags |= 0x01;
  if (kick)   cmd.flags |= 0x02;

  cmd.vx   = static_cast<int8_t>(-vy_rot); // swap because bottom pcb is wrong
  cmd.vy   = static_cast<int8_t>(-vx_rot);
  cmd.rot  = static_cast<int8_t>(rot);
  cmd.drib = static_cast<int8_t>(dribbler);

  Wire.beginTransmission(motorMBAddress);
  Wire.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
  Wire.endTransmission();
}
