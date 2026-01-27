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
#include <util/MovingAverage.h>

elapsedMillis hasBallTimer;
elapsedMillis ledTimer;
elapsedMillis positionYAvgTimer;

MovingAverage<double, 10> positionYAvg;

// y axis
double y_Setpoint = 0, y_Input = 0, y_Output = 0;
constexpr double y_Kp=1.4, y_Ki=0.00, y_Kd=0.05;
PID y_motion(&y_Input, &y_Output, &y_Setpoint, y_Kp, y_Ki, y_Kd, DIRECT);

// x axis
double x_Setpoint = 0, x_Input = 0, x_Output = 0;
constexpr double x_Kp=1.3, x_Ki=0.00, x_Kd=0.05;
PID x_motion(&x_Input, &x_Output, &x_Setpoint, x_Kp, x_Ki, x_Kd, DIRECT);

// rotation
double rot_Setpoint = 0, rot_Input = 0, rot_Output = 0;
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
  x_Setpoint = 0.0;
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

int Bot::getRotationControl(const float input) {
  rot_Input = input;
  if (std::abs(rot_Input) < 5) {
    rot_Input = 0;
  }
  rot_motion.Compute();
  return static_cast<int>(rot_Output);
}

Vector2 Bot::getAwayFromLineVec(const int speed) {
  Vector2 line = degreeToVector(_sensors->getLineRot());
  line.rotate(std::numbers::pi);

  Vector2 middlePointVector = _positioning->getMiddlePointVector();
  middlePointVector.normalize();

  line = line * 0.3f + middlePointVector * 0.7f;
  line.normalize();
  line *= speed;

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
  const double targetGoalRot = _cm5->getTargetGoalRot();
  Vector2 target = degreeToVector(targetGoalRot);
  target *= speed;
  return target;
}

Vector2 Bot::getBallApproachVec(const int speed) const {
  const double ballRot = _cm5->getBallRot();
  Vector2 target = degreeToVector(ballRot);
  target.normalize();
  target *= speed;
  return target;
}

Vector2 Bot::getBallPursuitVec() const {
  const double ballDist = _cm5->getBallDist();
  const double ballRot = _cm5->getBallRot();
  const double targetGoalRot = _cm5->getTargetGoalRot();
  const double targetGoalDist = _cm5->getTargetGoalDist();

  const auto ballVec = Vector2(
    cos(ballRot * (std::numbers::pi / 180.0f)) * ballDist,
    sin(ballRot * (std::numbers::pi / 180.0f)) * ballDist
  );

  const auto targetGoalVec = Vector2(
    cos(targetGoalRot * (std::numbers::pi / 180.0f)) * targetGoalDist,
    sin(targetGoalRot * (std::numbers::pi / 180.0f)) * targetGoalDist
  );

  // ball pursuit on straight between ball and goal
  Vector2 ballToGoal = targetGoalVec - ballVec;
  ballToGoal.normalize();

  constexpr double offsetDist = 20.0;
  Vector2 idealPos = ballVec - ballToGoal * offsetDist;

  Vector2 robotToIdeal = idealPos;
  robotToIdeal.normalize();

  Vector2 robotToBall = ballVec;;
  robotToBall.normalize();

  const double dot = robotToIdeal.getX() * robotToBall.getX() + robotToIdeal.getY() * robotToBall.getY();

  if (std::abs(ballRot) > 70.0 && std::abs(dot) > 0.6) {
    const Vector2 perpendicular(-ballToGoal.getY(), ballToGoal.getX());

    const double cross = ballVec.getX() * targetGoalVec.getY() - ballVec.getY() * targetGoalVec.getX();
    const double side = (cross > 0) ? 1.0 : -1.0;

    const double shiftStrength = std::clamp((dot - 0.5) * 2.0, 0.0, 1.0);
    constexpr double maxShift = 30.0;

    idealPos = idealPos + perpendicular * (side * maxShift * shiftStrength);
  }

  const Vector2 target = idealPos;
  return target;
}

void Bot::updateYMotion(const double y) const {
  y_Input = y;
  if (isnan(y_Input)) y_Input = 0;
  y_motion.Compute();
}

void Bot::updateXMotion(const double x) const {
  x_Input = x;
  if (isnan(x_Input)) x_Input = 0;
  x_motion.Compute();
}

void Bot::updatePositionYAvg(const double y) {
  if (positionYAvgTimer > 32) {
    positionYAvg.addValue(y);
    positionYAvgTimer = 0;
  }
}

bool Bot::checkBallOnLine() const {
  const double globalY = _cm5->getGlobalY();
  const double ballRot = _cm5->getBallRot();
  const double ballDist = _cm5->getBallDist();

  const double ballRadians = ballRot * (std::numbers::pi / 180.0);
  const double ballGlobalY = globalY + sin(ballRadians) * ballDist;

  if (globalY > 80.0 && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -80.0 && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

Vector2 Bot::dribbleBackwards(const int speed) const {
  auto mid = _positioning->getMiddlePointVector();
  mid.normalize();
  return mid * speed;
}

void Bot::update() {
  constexpr int speed = 50.0f;
  static bool CM5_initialized = false;
  static bool kickOff = false;

  _cm5->update();
  updatePositionYAvg(_cm5->getGlobalY());
  _sensors->update();
  _positioning->update();
  setRotDelta(_positioning->getRotationDelta());

  if (!kickOff && _sensors->getEna()) {
    pushData(_sensors->getEna(), false, 50, 0, 0, 0);
    delay(300);
    kickOff = true;
    pushData(_sensors->getEna(), true, 50, 0, 0, 0);
    delay(100);
  }

  if (!_cm5->getCM5Running()) {
    CM5_initialized = false;
    overrideControl();
    _sensors->haltLEDs();
    return;
  }
  if (_cm5->getCM5Running() != CM5_initialized) {
    CM5_initialized = !CM5_initialized;
    _sensors->allLEDsOff();
  }

  // toggle all leds off
  if (!_sensors->getEna()) {
    ledTimer = 0;
  }
  if (ledTimer > 200) {
    _sensors->allLEDsOff();
  }

  Vector2 target;
  double rotInput = 0;
  bool usePID = true;
  bool kick = false;

  if (!Sensors::getHasBall()) {
    hasBallTimer = 0;
  }
  // drive away from line
  if (_sensors->getLineSeen()) {
    target = getAwayFromLineVec(30);
    if (checkBallOnLine() && std::abs(_cm5->getBallRot()) < 90.0) {
      if (std::abs(_cm5->getHeading()) < 90.0) {
        rotInput = _cm5->getBallRot();
      }
      else {
        rotInput = _cm5->getHeading();
      }
    }
    else {
      rotInput = _cm5->getHeading();
    }
    usePID = false;
  }

  // drive to goal if has ball
  else if (Sensors::getHasBall()) {
    if (hasBallTimer > 50) {
      target = getBallAlignedVec(speed);
      rotInput = _cm5->getTargetGoalRot();

      if (std::abs(_cm5->getTargetGoalRot()) < 20.0 && _sensors->getEna()) {
        kick = true;
      }
    }
  }

  // drive to midPoint if ball not seen
  else if (!_cm5->getBallExists()) {
    target = getMoveToCenterVec(speed);
    rotInput = _cm5->getHeading();
    usePID = false;
  }

  // else pursue ball
  else {

    // approach ball
    if (std::abs(_cm5->getBallRot()) < 10 && _cm5->getBallDist() < 30.0) {
      target = getBallApproachVec(30);
      if (std::abs(_cm5->getHeading()) < 90.0) {
        rotInput = _cm5->getBallRot();
      }
      else {
        rotInput = _cm5->getHeading();
      }
      usePID = false;
    }

    // pursue ball
    else {

      // rotate to ball
      if (checkBallOnLine() && std::abs(_cm5->getBallRot()) < 90.0) {
        if (std::abs(_cm5->getHeading()) < 90.0) {
          rotInput = _cm5->getBallRot();
        }
        else {
          rotInput = _cm5->getHeading();
        }
        target = getBallApproachVec(15);
        usePID = false;
      }

      // normal ball pursuit
      else {
        rotInput = _cm5->getTargetGoalRot();
        target = getBallPursuitVec();
      }
    }
  }

  // update pids
  const int rot = getRotationControl(static_cast<float>(rotInput));

  updateXMotion(target.getX());
  updateYMotion(target.getY());

  float vx = 0;
  float vy = 0;

  if (usePID) {
    vx = static_cast<float>(-x_Output);
    vy = static_cast<float>(-y_Output);
  } else {
    vx = static_cast<float>(target.getX());
    vy = static_cast<float>(target.getY());
  }

  _positioning->speedLimit(vx, vy);

  pushData(_sensors->getEna(), kick, static_cast<int>(vx), static_cast<int>(vy), rot, speed);
}

void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}