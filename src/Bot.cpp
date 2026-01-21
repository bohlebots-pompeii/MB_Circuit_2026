//
// Created by julius on 11.11.2025.
//

#include <Bot.h>
#include <Arduino.h>
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
#include <PID_v1.h>

elapsedMillis lineLastSeen;
elapsedMillis yellowAligned;

Bot::Bot() {
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N2, 16, 17);

  _cm5 = std::make_shared<CM5>();
  _sensors = std::make_shared<Sensors>(_cm5);
  _positioning = std::make_shared<Positioning>(_cm5);

  double x_Setpoint, x_Input, x_Output;
  constexpr double Kp=0.0, Ki=0.0, Kd=0.0;
  PID x_motion(&x_Input, &x_Output, &x_Setpoint, Kp, Ki, Kd, DIRECT);
}

Vector2 degreeToVector(const double degrees) {
  const double radians = degrees * (PI / 180.0f);
  return Vector2(cos(radians), sin(radians));
}

int Bot::getRotationControl() const {
  const float heading = _cm5->getHeading();
  return 0 - static_cast<int>(heading) / 3;
}

Vector2 Bot::getAwayFromLineVec() {
  Vector2 line = degreeToVector(_sensors->getLineRot());
  line.rotate(std::numbers::pi);

  Vector2 middlePointVector = _positioning->getMiddlePointVector();
  middlePointVector.normalize();

  line = line * 0.3f + middlePointVector * 0.7f;
  line.normalize();
  line *= 30;

  lineLastSeen = 0;
  lastLine = line;
  return line;
}

Vector2 Bot::getMoveToCenterVec(int speed) const {
  Vector2 middlePointVector = _positioning->getMiddlePointVector();
  const double distance = middlePointVector.getMagnitude();
  middlePointVector.normalize();

  constexpr double MAX_DISTANCE = 30.0f;
  const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
  const double speedFactor = ratio * ratio;
  const int dynamicSpeed = static_cast<int>(speed * speedFactor);

  return middlePointVector * dynamicSpeed;
}

Vector2 Bot::getBallAlignedVec(int speed, int& rot) {
  const double yellowRot = _cm5->getYellowRot();
  Vector2 target = degreeToVector(yellowRot);

  if (abs(yellowRot) > 10) {
    yellowAligned = 0;
  }
  target.normalize();
  rot = static_cast<int>(yellowRot / 2);

  if (yellowAligned > 200) {
    return target * speed;
  }
  return Vector2(0, 0);
}

Vector2 Bot::getBallApproachVec(int speed) const {
  const double ballRot = _cm5->getBallRot();
  Vector2 target = degreeToVector(ballRot);
  target.normalize();
  return target * speed;
}

Vector2 Bot::getBallPursuitVec(int speed) const {
  const float heading = _cm5->getHeading();
  const double ballDist = _cm5->getBallDist();
  const double ballRot = _cm5->getBallRot();

  const auto ballVec = Vector2(
    cos(ballRot * (std::numbers::pi / 180.0f)) * ballDist,
    sin(ballRot * (std::numbers::pi / 180.0f)) * ballDist
  );

  Vector2 offsetVec = degreeToVector(heading);
  offsetVec.rotate(ballVec.getAngle() * 0.5);

  const double ballAngle = std::abs(ballVec.getAngle());
  const double ballAngleNorm = std::clamp(ballAngle / (std::numbers::pi / 2), 0.0, 1.0);

  constexpr double k = 1.12;
  const double smoothBallAngleNorm = std::pow(ballAngleNorm, k);

  constexpr double min = 0.70, max = 1.0;
  const double factor = min + ((max - min) * smoothBallAngleNorm);
  offsetVec *= factor * 17;

  Vector2 target = ballVec - offsetVec;
  const double magnitude = target.getMagnitude();
  const double clamped = std::clamp(magnitude, 30.0, static_cast<double>(speed));

  if (magnitude > 1e-6) {
    target *= clamped / magnitude;
  }
  return target;
}

void Bot::update() {
  constexpr int speed = 50;

  _cm5->update();
  _sensors->update();
  _positioning->update();

  int rot = getRotationControl();

  // Line avoidance
  if (_sensors->getLineSeen()) {
    Vector2 line = getAwayFromLineVec();
    pushData(_sensors->getEna(), false, static_cast<int>(line.getX()), static_cast<int>(line.getY()), rot, 0);
    return;
  }

  // No ball - move to center
  if (!_cm5->getBallExists()) {
    Vector2 center = getMoveToCenterVec(speed);
    pushData(_sensors->getEna(), false, static_cast<int>(center.getX()), static_cast<int>(center.getY()), rot, 100);
    return;
  }

  const double ballRot = _cm5->getBallRot();

  // Ball aligned - aim at goal
  if (abs(ballRot) < 10) {
    Vector2 target = getBallAlignedVec(speed, rot);
    pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
    return;
  }

  // Ball in front - approach directly
  if (abs(ballRot) < 80) {
    Vector2 target = getBallApproachVec(speed);
    pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
    return;
  }

  // Ball behind - pursuit maneuver
  Vector2 target = getBallPursuitVec(speed);
  pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}