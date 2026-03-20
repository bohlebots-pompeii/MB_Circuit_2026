#include <nodes/striker/GetBehindBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <config/config.h>
#include <cmath>
#include <algorithm>

GetBehindBall::GetBehindBall(std::shared_ptr<MotionController> motion)
  : _motion(std::move(motion)) {}

BT::Status GetBehindBall::tick(const WorldState& ws) {
  if (!(ws.ballExists && !ws.hasBall)) {
    return BT::Status::FAILURE;
  }

  Vector2 target;
  float rotInput = 0;
  bool usePID = true;
  int speed = 50;

  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  const bool ballAligned = std::abs(globalBallDir) < FieldConfig::rotateToBallAngle;
  const bool ballInEdgeCase = checkBallOnLine(ws) || checkBallInPocket(ws);

  if (std::abs(ws.ballRot) < 15.0) {
    // ball is straight ahead — direct approach
    target = getBallApproachVec(ws, ws.ballDist > 30.0 ? speed : 30);
    rotInput = ballAligned ? ws.ballRot : ws.heading;
    usePID = false;
  }
  else if (ballInEdgeCase && ballAligned) {
    // ball near line or pocket — careful approach
    target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);
    rotInput = ws.ballRot;
    usePID = false;
  }
  else {
    // normal pursuit
    target = getBallPursuitVec(ws);
    rotInput = ws.targetGoalRot;
    usePID = true;
  }

  if (ws.peerRunning && ws.globalX < -70) {
    // prevent crashing into each other
    Vector2 mv(-ws.globalX, -ws.globalY);
    mv.normalize();
    target = mv * 20.0f;
    usePID = false;
  }

  const int drib = target.getX() > 10 ? 50 : 100;

  // MotionController compute
  auto [vx, vy, rot] = _motion->compute(target, rotInput, usePID);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);

  return BT::Status::RUNNING;
}

Vector2 GetBehindBall::getBallPursuitVec(const WorldState& ws) const {
  const auto ballVec = ws.ballVec;
  const auto targetGoalVec = ws.targetGoalVec;

  // ball pursuit on straight between ball and goal
  Vector2 ballToGoal = targetGoalVec - ballVec;
  ballToGoal.normalize();

  constexpr double offsetDist = 20.0;
  Vector2 idealPos = ballVec - ballToGoal * offsetDist;

  Vector2 robotToIdeal = idealPos;
  robotToIdeal.normalize();

  Vector2 robotToBall = ballVec;
  robotToBall.normalize();

  if (const double dot = robotToIdeal.getX() * robotToBall.getX() + robotToIdeal.getY() * robotToBall.getY();
    std::abs(ws.ballRot) > 60.0 && std::abs(dot) > 0.6) {
    const Vector2 perpendicular(-ballToGoal.getY(), ballToGoal.getX());

    const double cross = ballVec.getX() * targetGoalVec.getY() - ballVec.getY() * targetGoalVec.getX();
    const double side = cross > 0 ? 1.0 : -1.0;

    const double shiftStrength = std::clamp((dot - 0.5) * 2.0, 0.0, 1.0);
    constexpr double maxShift = 30.0;

    idealPos = idealPos + perpendicular * (side * maxShift * shiftStrength);
  }

  Vector2 target = idealPos;
  if (target.getMagnitude() < 10.0) {
    target.normalize();
    target *= 10;
  }
  return target;
}

Vector2 GetBehindBall::getBallApproachVec(const WorldState& ws, int speed) const {
  auto target = ws.ballVec;
  target.normalize();
  return target * speed;
}

Vector2 GetBehindBall::getBallAlignedVec(const WorldState& ws, int speed) const {
  Vector2 target = degToVec(ws.targetGoalRot);
  target.normalize();
  return target * speed;
}

bool GetBehindBall::checkBallOnLine(const WorldState& ws) const {
  const double globalY = ws.globalY;
  const double ballDist = ws.ballDist;

  double globalBallRot = ws.ballRot - ws.heading;
  if (globalBallRot > 180.0) globalBallRot -= 360.0;
  if (globalBallRot < -180.0) globalBallRot += 360.0;

  const double ballRadians = toRad(globalBallRot);
  const double ballGlobalY = globalY + sin(ballRadians) * ballDist;

  if (globalY > FieldConfig::FieldLinePositionY && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -FieldConfig::FieldLinePositionY && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

bool GetBehindBall::checkBallInPocket(const WorldState& ws) const {
  const double ballX = ws.ballVec.getX();

  float globalGoalDir = ws.targetGoalRot - ws.heading;
  while (globalGoalDir > 180) globalGoalDir -= 360;
  while (globalGoalDir < -180) globalGoalDir += 360;

  if (!ws.ballExists && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -
    FieldConfig::FieldPocketAngle)) {
    return true;
  }

  if (ballX > 0 && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
    return true;
  }

  return false;
}
