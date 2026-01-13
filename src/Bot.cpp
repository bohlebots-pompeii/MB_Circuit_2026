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
  _positioning = std::make_shared<Positioning>(_cm5);
}

Vector2 degreeToVector(const float degrees) {
  const float radians = degrees * (PI / 180.0f);
  return Vector2(sinf(radians), cosf(radians));
}

void Bot::update() {
  int speed = 35;

  _cm5->update();
  _sensors->update();
  _positioning->update();

  // rotation motion control
  const float heading = _cm5->getHeading();
  int rot = 0 - static_cast<int>(heading) / 4;

  // --- Line Sensor Override Logic ---
  // If the line sensor detects the boundary, prioritize moving away
  if (_sensors->getLineSeen()) {
    // Calculate vector away from the line
    Vector2 line = degreeToVector(_sensors->getLineRot());
    line.rotate(std::numbers::pi);

    Vector2 middlePointVector = _positioning->getMiddlePointVector();
    middlePointVector.normalize();

    // Blend line avoidance with movement towards the center
    line = line * 0.3f + middlePointVector * 0.7f;
    line.normalize();

    const int vx_l = static_cast<int>(roundf(line.getX() * 20));
    const int vy_l = static_cast<int>(roundf(line.getY() * 20));

    lineLastSeen = 0;
    lastLine = line;

    pushData(_sensors->getEna(), false, vx_l, vy_l, rot, 0);
    return;
  }

  if (lineLastSeen < 100) {
    const int vy_l = static_cast<int>(roundf(lastLine.getX() * 20));
    const int vx_l = static_cast<int>(roundf(lastLine.getY() * 20));

    pushData(_sensors->getEna(), false, vx_l, vy_l, rot, 0);
    return;
  }

  if (!_cm5->getBallExists()) {
    // No ball detected, move towards the center
    Vector2 middlePointVector = _positioning->getMiddlePointVector();
    const float distance = middlePointVector.getMagnitude();
    middlePointVector.normalize();

    constexpr float MAX_DISTANCE = 30.0f;
    const float ratio = std::min(distance / MAX_DISTANCE, 1.0f);
    const float speedFactor = ratio * ratio;

    const int dynamicSpeed = static_cast<int>(speed * speedFactor);

    const int vx_c = static_cast<int>(roundf(middlePointVector.getX() * dynamicSpeed));
    const int vy_c = static_cast<int>(roundf(middlePointVector.getY() * dynamicSpeed));

    pushData(_sensors->getEna(), false, vx_c, vy_c, rot, 0);
    return;
  }

  // --- get Sensor Data ---
  int16_t ballDist = _cm5->getBallDist();
  const int16_t ballRot = _cm5->getBallRot();
  const int16_t yellow_rot = _cm5->getYellowRot();

  if (ballDist > 100) {
    ballDist = 100;
  }
  // --- Movement Logic ---
  Vector2 botPos = _positioning->getMiddlePointVector();
  botPos.rotate(std::numbers::pi);

  const Vector2 goalVec = degreeToVector(yellow_rot);
  // auto ballVec = Vector2(sinf(ballRot) * ballDist, cosf(ballRot) * ballDist);

  float shiftFactor = 25 / static_cast<float>(ballDist);
  shiftFactor = constrain(shiftFactor, 1.0f, 3.0f);

  float shift = ballRot * shiftFactor;
  shift = constrain(shift, -220.0f, 220.0f);
  Serial.println(shift);
  Vector2 shiftVec = degreeToVector(shift);
  shiftVec.normalize();

  shiftVec *= ballDist;

  Vector2 target = shiftVec;
  target.normalize();

  // If ball is roughly in front, align with the yellow goal
  if (abs(ballRot) < 10.0f) {
    target = degreeToVector(yellow_rot);
     rot = 0 - yellow_rot / 2;
  }

  // speed = _positioning->speedLimit(target, speed);

  // Convert target vector to motor velocities (swap X/Y for omni kinematics)
  const int vx = static_cast<int>(roundf(target.getX() * speed));
  const int vy = static_cast<int>(roundf(target.getY() * speed));

  pushData(_sensors->getEna(), false, static_cast<int>(roundf(vx)), static_cast<int>(roundf(vy)), rot, 0);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}