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
elapsedMillis ballLastSeen;

// y axis
double y_Setpoint, y_Input, y_Output;
constexpr double y_Kp=0.7, y_Ki=0.00, y_Kd=0.06;
PID y_motion(&y_Input, &y_Output, &y_Setpoint, y_Kp, y_Ki, y_Kd, DIRECT);

// x axis
double x_Setpoint, x_Input, x_Output;
constexpr double x_Kp=1.0, x_Ki=0.00, x_Kd=0.05;
PID x_motion(&x_Input, &x_Output, &x_Setpoint, x_Kp, x_Ki, x_Kd, DIRECT);

// rotation
double rot_Setpoint, rot_Input, rot_Output;
constexpr double rot_Kp=0.5, rot_Ki=0.0, rot_Kd=0.05;
PID rot_motion(&rot_Input, &rot_Output, &rot_Setpoint, rot_Kp, rot_Ki, rot_Kd, DIRECT);

Bot::Bot() {
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(921600, SERIAL_8N2, 16, 17);

  _cm5 = std::make_shared<CM5>();
  _sensors = std::make_shared<Sensors>(_cm5);
  _positioning = std::make_shared<Positioning>(_cm5);

  y_Setpoint = 0.0;
  x_Setpoint = 20.0;
  rot_Setpoint = 0.0;

  rot_motion.SetMode(AUTOMATIC);
  rot_motion.SetOutputLimits(-50, 50);
  rot_motion.SetSampleTime(30);

  y_motion.SetMode(AUTOMATIC);
  y_motion.SetOutputLimits(-50, 50);
  y_motion.SetSampleTime(30);

  x_motion.SetMode(AUTOMATIC);
  x_motion.SetOutputLimits(-50, 50);
  x_motion.SetSampleTime(30);
}

Vector2 degreeToVector(const double degrees) {
  const double radians = degrees * (PI / 180.0f);
  return Vector2(cos(radians), sin(radians));
}

int Bot::getRotationControl() const {
  rot_Input = _cm5->getHeading();
  if (abs(rot_Input) < 5) {
    rot_Input = 0;
  }
  rot_motion.Compute();
  return static_cast<int>(rot_Output);
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

Vector2 Bot::getMoveToCenterVec(const int speed) const {
  Vector2 middlePointVector = _positioning->getMiddlePointVector();
  const double distance = middlePointVector.getMagnitude();
  middlePointVector.normalize();

  constexpr double MAX_DISTANCE = 30.0f;
  const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
  const double speedFactor = ratio * ratio;
  const int dynamicSpeed = static_cast<int>(speed * speedFactor);

  return middlePointVector * dynamicSpeed;
}

Vector2 Bot::getBallAlignedVec(const int speed) const {
  const double yellowRot = _cm5->getYellowRot();
  Vector2 target = degreeToVector(yellowRot);
  target *= speed;
  rot_Setpoint = yellowRot * 2;
  return target;
}

Vector2 Bot::getBallApproachVec(const int speed) const {
  const double ballRot = _cm5->getBallRot();
  Vector2 target = degreeToVector(ballRot);
  target.normalize();
  target *= speed;
  target.setY(-y_Output);
  return target;
}

Vector2 Bot::getBallPursuitVec(const int speed) const {
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

  constexpr double k = 0.9;
  const double smoothBallAngleNorm = std::pow(ballAngleNorm, k);

  constexpr double min = 0.75, max = 1.0;
  const double factor = min + ((max - min) * smoothBallAngleNorm);
  offsetVec *= factor * 20;

  Vector2 target = ballVec - offsetVec;
  const double magnitude = target.getMagnitude();
  const double clamped = std::clamp(magnitude, 30.0, static_cast<double>(speed));

  if (magnitude > 1e-6) {
    target *= clamped / magnitude;
  }

  if (std::abs(ballRot) < 70) {
    target.setY(-y_Output);
  }
  return target;
}

void Bot::updateYMotion(const double y) const {
  const double ballRot = _cm5->getBallRot();
  y_Input = ballRot;
  y_motion.Compute();
}

void Bot::updateXMotion(const double x) const {
  const double ballDist = _cm5->getBallDist();
  x_Input = ballDist;
  x_motion.Compute();
}

void Bot::update() {
  constexpr int speed = 40;

  _cm5->update();
  _sensors->update();
  _positioning->update();

  // updateXMotion(0);
  updateYMotion(0);

  const int rot = getRotationControl();

  // Line avoidance
  if (_sensors->getLineSeen()) {
    const Vector2 line = getAwayFromLineVec();
    pushData(_sensors->getEna(), false, static_cast<int>(line.getX()), static_cast<int>(line.getY()), rot, 100);
    return;
  }

  if (_sensors->getHasBall()) {
    const Vector2 target = getBallAlignedVec(speed);
    pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
    return;
  }
  rot_Setpoint = 0.0;

  // No ball - move to center
  if (!_cm5->getBallExists()) {
    const Vector2 center = getMoveToCenterVec(speed);
    pushData(_sensors->getEna(), false, static_cast<int>(center.getX()), static_cast<int>(center.getY()), rot, 100);
    return;
  }

  // Ball in front - approach directly
  if (const double ballRot = _cm5->getBallRot(); abs(ballRot) < 35) {
    const Vector2 target = getBallApproachVec(speed);
    pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
    return;
  }

  // Ball behind - pursuit maneuver
  const Vector2 target = getBallPursuitVec(speed);
  pushData(_sensors->getEna(), false, static_cast<int>(target.getX()), static_cast<int>(target.getY()), rot, 100);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}