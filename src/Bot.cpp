//
// Created by julius on 11.11.2025.
//

#include <Bot.h>
#include <Arduino.h>
#include <chrono>
#include <Vector2.hpp>
#include <comms/CM5.h>
#include <Sensors.h>
#include <Positioning.h>
#include <Wire.h>
#include <memory>
#include <motor_mb.h>
#include <numbers>
#include <iostream>
#include <elapsedMillis.h>

elapsedMillis lineLastSeen;

Bot::Bot() {
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N2, 16, 17);

  _cm5 = std::make_shared<CM5>();
  _sensors = std::make_shared<Sensors>(_cm5);
  _positioning = std::make_shared<Positioning>(_sensors);
}

Vector2 degreeToVector(const float degrees) {
  const float radians = degrees * (PI / 180.0f);
  return Vector2(cosf(radians), sinf(radians));
}

void Bot::update() {
  constexpr int speed = 30;

  _cm5->update();
  _sensors->update();

  // Check homing state first to override standard gameplay logic
  if (isHoming) {
    home();
    return;
  }

  int rot = 0 - static_cast<int>(_cm5->getHeading()) / 4;

  // --- Line Sensor Override Logic ---
  // If the line sensor detects the boundary, prioritize moving away
  if (_sensors->getLineSeen()) {
    // Calculate vector away from the line (rotate 180 degrees from line normal)
    Vector2 line = degreeToVector(_sensors->getLineRot());
    line.normalize();
    line.rotate(std::numbers::pi);

    const int vx_l = static_cast<int>(roundf(line.getY() * 10));
    const int vy_l = static_cast<int>(roundf(line.getX() * 10));

    lineLastSeen = 0;
    lastLine = line;

    pushData(_sensors->getEna(), false, vx_l, vy_l, 0, 0);
    return;
  }
  if (lineLastSeen < 100) {
    const int vy_l = static_cast<int>(roundf(lastLine.getX() * 10));
    const int vx_l = static_cast<int>(roundf(lastLine.getY() * 10));

    pushData(_sensors->getEna(), false, vx_l, vy_l, 0, 0);
    return;
  }

  // --- Ball Tracking Logic ---
  int16_t ballDist = _cm5->getBallDist();
  const int16_t ballRot = _cm5->getBallRot();
  const int16_t yellow_rot = _cm5->getYellowRot();

  if (ballDist > 100) {
    ballDist = 100;
  }

  // Calculate orbital shift to curve behind the ball
  float shift;
  if (ballDist != 0 && abs(ballRot) > 40.0f) {
    shift = 16.0f / (ballDist / 2.0f);
    shift = constrain(shift, 1.0, 3.0);
  }
  else {
    shift = 1.0f;
  }

  Vector2 target = degreeToVector(ballRot * shift);
  target.normalize();

  // If ball is roughly in front, align with the yellow goal
  if (abs(ballRot) < 15.0f) {
    target = degreeToVector(yellow_rot);
     rot = 0 - -yellow_rot / 2;
  }

  // Convert target vector to motor velocities (swap X/Y for omni kinematics)
  const int vx = static_cast<int>(roundf(target.getY() * speed));
  const int vy = static_cast<int>(roundf(target.getX() * speed));

  pushData(_sensors->getEna(), false, static_cast<int>(roundf(vx)), static_cast<int>(roundf(vy)), rot, 0);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}

void Bot::home() {
  constexpr float TARGET_X = -50.0f;
  constexpr float TARGET_Y = -90.0f;
  constexpr float KP_POS = 1.5f;
  constexpr float KP_ROT = 0.8f;
  constexpr float MAX_SPEED = 40.0f;
  constexpr float MAX_ROT_SPEED = 50.0f;

  const Vector2 pos = _sensors->getPosition();
  const float error_x = TARGET_X - pos.getX();
  const float error_y = TARGET_Y - pos.getY();
  const float distance = sqrtf(error_x * error_x + error_y * error_y);

  if (constexpr float GOAL_RADIUS = 3.0f; distance < GOAL_RADIUS) {
    isHoming = false;
    pushData(false, false, 0, 0, 0, 0);
    return;
  }

  float world_vx = error_x * KP_POS;
  float world_vy = error_y * KP_POS;

  if (const float speed = sqrtf(world_vx * world_vx + world_vy * world_vy); speed > MAX_SPEED) {
    world_vx = world_vx / speed * MAX_SPEED;
    world_vy = world_vy / speed * MAX_SPEED;
  }

  const float heading = _cm5->getHeading();
  const float theta = heading * (PI / 180.0f);
  const float cos_theta = cosf(theta);
  const float sin_theta = sinf(theta);

  const float local_vx = cos_theta * world_vx + sin_theta * world_vy;
  const float local_vy = -sin_theta * world_vx + cos_theta * world_vy;

  float rot_speed = 0 - heading / 4;
  rot_speed = constrain(rot_speed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

  // Use sensor enable flag instead of hardcoded true for safety
  pushData(_sensors->getEna(), false, static_cast<int>(local_vx), static_cast<int>(local_vy), static_cast<int>(rot_speed), 0);
}