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
#include <util/WebDebugger.h>

// y axis
double y_Setpoint = 0, y_Input = 0, y_Output = 0;
constexpr double y_Kp=1.4, y_Ki=0.00, y_Kd=0.03;
PID y_motion(&y_Input, &y_Output, &y_Setpoint, y_Kp, y_Ki, y_Kd, DIRECT);

// x axis
double x_Setpoint = 0, x_Input = 0, x_Output = 0;
constexpr double x_Kp=1.2, x_Ki=0.00, x_Kd=0.03;
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

Vector2 Bot::getAwayFromLineVec() {
  Vector2 line = degreeToVector(_sensors->getLineRot());
  line.rotate(std::numbers::pi);

  Vector2 middlePointVector = _positioning->getMiddlePointVector();
  middlePointVector.normalize();

  line = line * 0.3f + middlePointVector * 0.7f;
  line.normalize();
  line *= 30;

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
  return target;
}

Vector2 Bot::getBallApproachVec(const int speed) const {
  const double ballRot = _cm5->getBallRot();
  Vector2 target = degreeToVector(ballRot);
  target.normalize();
  target *= speed;
  return target;
}

Vector2 Bot::getBallPursuitVec(const int speed) const {
  const float heading = _cm5->getHeading();
  const double ballDist = _cm5->getBallDist();
  const double ballRot = _cm5->getBallRot();
  const double yellowRot = _cm5->getYellowRot();
  const double yellowDist = _cm5->getYellowDist();

  const auto ballVec = Vector2(
    cos(ballRot * (std::numbers::pi / 180.0f)) * ballDist,
    sin(ballRot * (std::numbers::pi / 180.0f)) * ballDist
  );

  const auto yellowVec = Vector2(
    cos(yellowRot * (std::numbers::pi / 180.0f)) * yellowDist,
    sin(yellowRot * (std::numbers::pi / 180.0f)) * yellowDist
  );

  // ball pursuit on straight between ball and goal
  Vector2 ballToGoal = yellowVec - ballVec;
  ballToGoal.normalize();

  constexpr double offsetDist = 20.0;
  Vector2 idealPos = ballVec - ballToGoal * offsetDist;

  Vector2 robotToIdeal = idealPos;
  robotToIdeal.normalize();

  Vector2 robotToBall = ballVec;;
  robotToBall.normalize();

  const double dot = robotToIdeal.getX() * robotToBall.getX() + robotToIdeal.getY() * robotToBall.getY();

  if (std::abs(ballRot) > 70.0 || std::abs(dot) > 0.6) {
    const Vector2 perpendicular(-ballToGoal.getY(), ballToGoal.getX());

    const double cross = ballVec.getX() * yellowVec.getY() - ballVec.getY() * yellowVec.getX();
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

void Bot::update() {
  constexpr int speed = 50.0f;

  _cm5->update();
  _sensors->update();
  _positioning->update();
  setRotDelta(_positioning->getRotationDelta());

  Vector2 target;
  double rotInput = 0;
  bool usePID = true;
  bool kick = false;

  if (_sensors->getLineSeen()) {
    target = getAwayFromLineVec();
    rotInput = _cm5->getHeading();
    usePID = false;
  }
  else if (_sensors->getHasBall()) {
    if (std::abs(_cm5->getYellowRot()) < 5) {
      kick = true;
    }
    target = getBallAlignedVec(speed);
    rotInput = _cm5->getYellowRot() / 1.5;
  }
  else if (!_cm5->getBallExists()) {
    target = getMoveToCenterVec(speed);
    rotInput = _cm5->getHeading();
    usePID = false;
  }
  else {
    const double ballRot = _cm5->getBallRot();

    if (std::abs(ballRot) < 10 && _cm5->getBallDist() < 30) {
      Serial.println("Approachng ball");
      target = getBallApproachVec(20);
      rotInput = _cm5->getBallRot();
      usePID = false;
    }
    else {
      Serial.println("Pursuit");
      target = getBallPursuitVec(speed);
      rotInput = _cm5->getYellowRot();
    }
  }

  // update pids
  const int rot = getRotationControl(static_cast<float>(rotInput));

  updateXMotion(target.getX());
  updateYMotion(target.getY());


  if (usePID) {
    pushData(_sensors->getEna(), kick, static_cast<int>(-x_Output), static_cast<int>(-y_Output), rot, 100);
  } else {
    pushData(_sensors->getEna(), kick, target.getX(), target.getY(), rot, 100);
  }
}


void Bot::overrideControl() {
  pushData(false, false, 0, 0, 0, 0);
}

void Bot::initDebugger(const char* ssid, const char* password) {
  webDebugger.begin(ssid, password);
}

void Bot::initDebuggerAP(const char* ssid, const char* password) {
  webDebugger.beginAP(ssid, password);
}

void Bot::sendDebugData(const Vector2& target) const {
  DebugData data;

  // Target vector
  data.target = target;

  // Ball vector
  const double ballRot = _cm5->getBallRot();
  const double ballDist = _cm5->getBallDist();
  data.ballVec = Vector2(
    cos(ballRot * (std::numbers::pi / 180.0)) * ballDist,
    sin(ballRot * (std::numbers::pi / 180.0)) * ballDist
  );

  // Goal vector
  const double yellowRot = _cm5->getYellowRot();
  const double yellowDist = _cm5->getYellowDist();
  data.goalVec = Vector2(
    cos(yellowRot * (std::numbers::pi / 180.0)) * yellowDist,
    sin(yellowRot * (std::numbers::pi / 180.0)) * yellowDist
  );

  // Middle point vector
  data.middlePointVec = _positioning->getMiddlePointVector();

  // Line vector
  data.lineVec = lastLine;

  // Scalars
  data.heading = _cm5->getHeading();
  data.ballRot = static_cast<float>(ballRot);
  data.ballDist = static_cast<float>(ballDist);
  data.yellowRot = static_cast<float>(yellowRot);
  data.yellowDist = static_cast<float>(yellowDist);
  data.blueRot = _cm5->getBlueRot();
  data.blueDist = _cm5->getBlueDist();
  data.globalX = _cm5->getGlobalX();
  data.globalY = _cm5->getGlobalY();

  // States
  data.hasBall = _sensors->getHasBall();
  data.lineSeen = _sensors->getLineSeen();
  data.ballExists = _cm5->getBallExists();

  // PID outputs
  data.xOutput = static_cast<float>(-x_Output);
  data.yOutput = static_cast<float>(-y_Output);
  data.rotOutput = static_cast<float>(rot_Output);

  webDebugger.setData(data);
  webDebugger.update();
}