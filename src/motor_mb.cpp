//
// Created by julius on 05.01.2026.
//

#include "motor_mb.h"
#include <Arduino.h>
#include <Wire.h>
#include <config/config.h>
#include <util/Vector2.hpp>

#include "MotionController.h"

// Target Buffers
static int _target_vx = 0;
static int _target_vy = 0;
static int _rot = 0;
static int _drib = 0;
static bool _ena = false;
static bool _kick = false;
static bool _useRotDelta = false;

// Ramping Buffers
static float _current_vx = 0.0f;
static float _current_vy = 0.0f;

// Ramp factor
constexpr float RAMP_FACTOR = 0.15f;

void pushData(const bool enable, const bool kick, const int vx, const int vy, const int rot, const int dribbler,
              const bool useRotDelta) {
  _target_vx = vx;
  _target_vy = vy;
  _rot = rot;
  _drib = dribbler;
  _ena = enable;
  _kick = kick;
  _useRotDelta = useRotDelta;
}

void setData(const int vx, const int vy, const int rot, const bool useRotDelta) {
  _target_vx = vx;
  _target_vy = vy;
  _rot = rot;
  _useRotDelta = useRotDelta;
}

void setDribbler(const int speed) {
  _drib = speed;
}

void setKick(const bool kick) {
  _kick = kick;
}

void setEnable(const bool enable) {
  _ena = enable;
}

void sendData() {
  MotorCmd cmd{};

  _current_vx += (static_cast<float>(_target_vx) - _current_vx) * RAMP_FACTOR;
  _current_vy += (static_cast<float>(_target_vy) - _current_vy) * RAMP_FACTOR;

  const int rotCalc = _rot * -1;

  Vector2 global_drive(_current_vx, _current_vy);
  if (_useRotDelta) {
    const double rotDeltaRad = MotionController::getInstance() ? MotionController::getInstance()->getRotDeltaRad() : 0.0;
    global_drive.rotate(-rotDeltaRad * 2.0f);
  }

  const int vx_rot = constrain(static_cast<int>(global_drive.getX()), -70, 70);
  const int vy_rot = constrain(static_cast<int>(global_drive.getY()), -70, 70);
  const int rot_final = constrain(rotCalc, -50, 50);
  const int dribbler_final = constrain(_drib, -30, 30);

  cmd.flags = 0;
  if (_ena) cmd.flags |= 0x01;
  if (_kick) cmd.flags |= 0x02;

  cmd.vx = static_cast<int8_t>(-vy_rot);
  cmd.vy = static_cast<int8_t>(-vx_rot);
  cmd.rot = static_cast<int8_t>(rot_final);
  cmd.drib = static_cast<int8_t>(dribbler_final);

  Wire.beginTransmission(I2C_ADDRESSES::MOTOR_MB_ADDR); // final send to execute
  Wire.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
  Wire.endTransmission();
  _kick = false;
}
