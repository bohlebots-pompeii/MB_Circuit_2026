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
  return Vector2(cosf(radians), sinf(radians));
}

void Bot::update() {
  int speed = 40;

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
  pushData(_sensors->getEna(), false, 0, 0, rot, 0);

  // --- get Sensor Data ---
  double ballDist = _cm5->getBallDist();
  double ballRot = _cm5->getBallRot();
  ballRot = ballRot * (std::numbers::pi / 180.0); // to rad

  if (ballDist > 100) {
    ballDist = 100;
  }

  // --- movement Logic ---
  const auto ballVec = Vector2(cosf(ballRot) * ballDist, sinf(ballRot) * ballDist);

  Vector2 offsetVec = degreeToVector(heading);
  const double a = ballVec.getAngle() * 0.5;
  offsetVec.rotate(a);

  const double ballAngle = std::abs(ballVec.getAngle());
  const double ballAngleNorm = std::clamp(ballAngle / (std::numbers::pi / 2), 0.0, 1.0);

  const double smoothBallAngleNorm = ballAngleNorm * ballAngleNorm * (3.0 - 2.0 * ballAngleNorm);

  constexpr double minFactor = 0.65;
  constexpr double maxFactor = 1.0;
  const double factorBallAngle = minFactor + ((maxFactor-minFactor) * smoothBallAngleNorm);

  offsetVec *= factorBallAngle * 25;

  Vector2 target = ballVec - offsetVec;
  const int magnitude = target.getMagnitude();
  const double c_magnitude = std::clamp(magnitude, 30, speed);
  target.normalize();
  target *= c_magnitude;

  /*
  Serial.print(target.getX());
  Serial.print(" | ");
  Serial.print(target.getY());
  Serial.print(" | ");
  Serial.print(target.getAngle());
  Serial.print(" | ");
  Serial.println(target.getMagnitude());
  */

  // speed = _positioning->speedLimit(target, speed);

  const int vx = static_cast<int>(roundf(target.getX()));
  const int vy = static_cast<int>(roundf(target.getY()));

  pushData(_sensors->getEna(), false, static_cast<int>(roundf(vx)), static_cast<int>(roundf(vy)), rot, 0);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}