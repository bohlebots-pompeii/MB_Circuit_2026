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

  pushData(_sensors->getEna(), false, 0, 0, rot, 100);

  /*

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

    pushData(_sensors->getEna(), false, vx_c, vy_c, rot, 100);
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
  const Vector2 goalVec = yellowVec;

  // --- compute decider ---
  Vector2 ballDir = ballVec;
  ballDir.normalize();
  Vector2 goalDir = goalVec;
  goalDir.normalize();

  const double dot = ballDir.getX() * goalDir.getX() + ballDir.getY() * goalDir.getY();

  if (abs(dot) >= 0.96 && abs(ballRot) < 90) {
    Vector2 target = degreeToVector(yellowRot);
    if (abs(yellowRot) > 10) {
      yellowAligned = 0;
    }
    target.normalize();
    rot = yellowRot / 2;

    int vx = 0;
    int vy = 0;
    if (yellowAligned > 200) {
      vx = static_cast<int>(roundf(target.getX() * speed));
      vy = static_cast<int>(roundf(target.getY() * speed));
    }

    pushData(_sensors->getEna(), false, vx, vy, rot, 100);
    return;
  }

  if (abs(dot) > 0.75 && abs(ballRot) < 90) {
    Vector2 target = degreeToVector(ballRot);
    target.normalize();

    const double ballLimit = constrain(yellowRot, -30, 30);
    // rot = ballLimit / 2;

    const int vx = static_cast<int>(roundf(target.getX() * speed));
    const int vy = static_cast<int>(roundf(target.getY() * speed));

    pushData(_sensors->getEna(), false, vx, vy, rot, 100);
    return;
  }

  //
  // ball pursiut
  //
  constexpr double angleNormMax = std::numbers::pi / 2;
  int vx = 0;
  int vy = 0;

  // when behind ball
  /*
  if (abs(ballRot) < 80) {
    constexpr double minBehindDist = 15.0;
    constexpr double maxBehindDist = 30.0;
    constexpr double smoothK = 1.4;

    Vector2 ballToGoal = goalVec - ballVec;
    ballToGoal.normalize();

    const double angle = std::acos(std::clamp(dot, -1.0, 1.0));

    const double angleNorm = std::clamp(angle / angleNormMax, 0.0, 1.0);
    const double smoothAngleNorm = std::pow(angleNorm, smoothK);

    const double behindDist = minBehindDist + (maxBehindDist - minBehindDist) * smoothAngleNorm;
    ballToGoal *= behindDist;

    Vector2 target = ballVec - ballToGoal;

    const double magnitude = target.getMagnitude();
    const double clamped = std::clamp(magnitude, 30.0, static_cast<double>(speed));

    if (magnitude > 1e-6) {
      target *= clamped / magnitude;
    }

    vx = static_cast<int>(std::round(target.getX()));
    vy = static_cast<int>(std::round(target.getY()));
  } else
    Vector2 offsetVec = degreeToVector(heading);
    const double a = ballVec.getAngle() * 0.5;
    offsetVec.rotate(a);

    const double ballAngle = std::abs(ballVec.getAngle());
    const double ballAngleNorm = std::clamp(ballAngle / (std::numbers::pi / 2), 0.0, 1.0);

    //const double smoothBallAngleNorm = ballAngleNorm * ballAngleNorm * (3.0 - 2.0 * ballAngleNorm);
    constexpr double k = 1.12;
    const double smoothBallAngleNorm = std::pow(ballAngleNorm, k);

    constexpr double min = 0.70;
    constexpr double max = 1.0;
    const double factor = min + ((max-min) * smoothBallAngleNorm);

    offsetVec *= factor * 17;

    Vector2 target = ballVec - offsetVec;

    const double magnitude = target.getMagnitude();
    const double clamped = std::clamp(magnitude, 30.0, static_cast<double>(speed));

    if (magnitude > 1e-6) {
      target *= clamped / magnitude;
    }

    vx = static_cast<int>(roundf(target.getX()));
    vy = static_cast<int>(roundf(target.getY()));

  pushData(_sensors->getEna(), false, vx, vy, rot, 100);
  */
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}