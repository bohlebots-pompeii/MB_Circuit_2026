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
elapsedMillis yellowAligned;

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
  return Vector2(cosf(radians), sinf(radians));
}

void Bot::update() {
  constexpr int speed = 50;

  _cm5->update();
  _sensors->update();
  _positioning->update();

  // rotation motion control
  const float heading = _cm5->getHeading();
  int rot = 0 - static_cast<int>(heading) / 3;

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

    const int vx_l = static_cast<int>(roundf(line.getX() * 30));
    const int vy_l = static_cast<int>(roundf(line.getY() * 30));

    lineLastSeen = 0;
    lastLine = line;

    pushData(_sensors->getEna(), false, vx_l, vy_l, rot, 0);
    return;
  }

  /*
  if (lineLastSeen < 100) {
    const int vy_l = static_cast<int>(roundf(lastLine.getX() * 20));
    const int vx_l = static_cast<int>(roundf(lastLine.getY() * 20));

    pushData(_sensors->getEna(), false, vx_l, vy_l, rot, 0);
    return;
  }
  */

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
  const double ballDist = _cm5->getBallDist();
  const double ballRot = _cm5->getBallRot();

  const double yellowDist = _cm5->getYellowDist();
  const double yellowRot = _cm5->getYellowRot();

  // --- movement Logic ---
  const auto ballVec = Vector2(cosf(ballRot * (std::numbers::pi / 180.0f)) * ballDist, sinf(ballRot * (std::numbers::pi / 180.0f)) * ballDist);
  const auto yellowVec = Vector2(cosf(yellowRot * (std::numbers::pi / 180.0f)) * yellowDist, sinf(yellowRot * (std::numbers::pi / 180.0f)) * yellowDist);

  if (abs(ballRot) < 10) {
    Vector2 goal = degreeToVector(yellowRot);
    if (abs(yellowRot) > 10) {
      yellowAligned = 0;
    }
    goal.normalize();
    rot = yellowRot / 2;

    int vx = 0;
    int vy = 0;
    if (yellowAligned > 200) {
      vx = static_cast<int>(roundf(goal.getX() * speed));
      vy = static_cast<int>(roundf(goal.getY() * speed));
    }

    pushData(_sensors->getEna(), false, vx, vy, rot, 0);
    return;
  }

  if (abs(ballRot) < 30) {
    Vector2 target = degreeToVector(ballRot);
    target.normalize();

    const double ballLimit = constrain(yellowRot, -30, 30);
    // rot = ballLimit / 2;

    const int vx = static_cast<int>(roundf(target.getX() * speed));
    const int vy = static_cast<int>(roundf(target.getY() * speed));

    pushData(_sensors->getEna(), false, vx, vy, rot, 0);
    return;
  }

  // drive behind ball
  Vector2 offsetVec = degreeToVector(heading);
  const double a = ballVec.getAngle() * 0.5;
  offsetVec.rotate(a);

  const double ballAngle = std::abs(ballVec.getAngle());
  const double ballAngleNorm = std::clamp(ballAngle / (std::numbers::pi / 2), 0.0, 1.0);

  //const double smoothBallAngleNorm = ballAngleNorm * ballAngleNorm * (3.0 - 2.0 * ballAngleNorm);
  constexpr double k = 0.9;          // >1 softer, <1 sharper
  const double smoothBallAngleNorm = std::pow(ballAngleNorm, k);

  constexpr double min = 0.80;
  constexpr double max = 1.0;
  const double factor = min + ((max-min) * smoothBallAngleNorm);

  offsetVec *= factor * 15;

  Vector2 target = ballVec - offsetVec;

  const double magnitude = target.getMagnitude();
  const double clamped = std::clamp(magnitude, 30.0, static_cast<double>(speed));

  if (magnitude > 1e-6) {
    target *= clamped / magnitude;
  }

  const int vx = static_cast<int>(roundf(target.getX()));
  const int vy = static_cast<int>(roundf(target.getY()));

  pushData(_sensors->getEna(), false, vx, vy, rot, 0);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}