#include <nodes/striker/GetBehindBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <config/config.h>
#include <cmath>
#include <algorithm>

static Vector2 getBallPursuitVec(const WorldState& ws) {
  const auto ballVec = ws.ballVec;
  const auto targetGoalVec = ws.targetGoalVec;

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

  const Vector2 target = idealPos;
  return target;
}

static Vector2 getBallApproachVec(const WorldState& ws, const int speed) {
  auto target = ws.ballVec;
  target.normalize();
  return target * speed;
}

static bool checkBallOnLine(const WorldState& ws) {
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

static bool checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  return std::abs(absoluteGoalDir) > FieldConfig::FieldPocketAngle;
}

void executeGetBehindBall(const WorldState& ws, MotionController* motion) {
  Vector2 target;
  double rotInput = 0;
  bool usePID;

  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  const bool ballAligned = std::abs(globalBallDir) < FieldConfig::rotateToBallAngle;
  const bool ballInEdgeCase = checkBallOnLine(ws) || checkBallInPocket(ws);

  if (ballAligned) {
    target = getBallApproachVec(ws, ws.ballDist > 30.0 ? 50 : 30);
    rotInput = ws.ballRot;
    usePID = false;
  }
  else if (ballInEdgeCase) {
    target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);
    rotInput = ws.ballRot;
    usePID = false;
  }
  else {
    // normal pursuit
    target = getBallPursuitVec(ws);
    rotInput = ws.heading;
    usePID = true;
  }

  if (ws.peerRunning && ws.globalX < -70) {
    // prevent crashing into each other
    Vector2 avoidVec(-ws.globalX, -ws.globalY);
    avoidVec.normalize();
    target = avoidVec * 20.0f;
    usePID = false;
  }

  constexpr int dribblerSpeed = 100;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, true);
}
