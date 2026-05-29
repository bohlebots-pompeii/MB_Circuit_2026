#include <nodes/striker/GetBehindBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <config/config.h>
#include <cmath>
#include <algorithm>
#include <numbers>

// helper
inline double wrapAngleRad(double a) {
  while (a > std::numbers::pi) a -= std::numbers::pi * 2;
  while (a < -std::numbers::pi) a += std::numbers::pi * 2;
  return a;
}

static Vector2 getBallPursuitVec(const WorldState& ws) {
  constexpr double circleRadius = 27.0;
  constexpr double driveForwardStart = 35.0;
  constexpr double driveForwardMax = 5.0;
  constexpr double throughDist = 10.0;
  constexpr double backwardsMultiplierStart = 135.0;

  const Vector2 ballVec = ws.ballVec;

  // default ball approach
  auto approachVec = Vector2(-ballVec.getY(), ballVec.getX());
  approachVec.normalize();

  // determine which side we drive around ball
  const double side = ballVec.getY() >= 0 ? 1.0 : -1.0;

  const Vector2 circlePoint = approachVec * circleRadius * side;

  // base target
  Vector2 target = ballVec + circlePoint;

  // clamp base target to make sure no overshoot
  if (ballVec.getX() > 0 && std::abs(target.getY()) > std::abs(ballVec.getY()) && target.getY() != 0) {
    const double scale = std::abs(ballVec.getY() / target.getY());
    target *= scale;
  }

  // linear interpolate to the throughpoint
  const auto throughPoint = Vector2(ballVec.getX() + throughDist, ballVec.getY());
  if (std::abs(ws.ballRot) < driveForwardStart) {
    const double t = std::clamp(
      (driveForwardStart - std::abs(ws.ballRot)) /
      (driveForwardStart - driveForwardMax),
      0.0,
      1.0
    );

    target = Vector2::lerp(target, throughPoint, t);
  }

  if (std::abs(ws.ballRot) > driveForwardStart && std::abs(ws.ballRot) < backwardsMultiplierStart) {
    const double t = std::clamp(
      (driveForwardStart - std::abs(ws.ballRot)) /
      (driveForwardStart - backwardsMultiplierStart),
      1.5,
      2.0
    );
    target.setX(target.getX() * t);
  }

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

  if (globalY > FieldConfig::LINE_POS_Y && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -FieldConfig::LINE_POS_Y && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

static bool checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  return std::abs(absoluteGoalDir) > FieldConfig::IN_POCKET_ANGLE;
}

void executeGetBehindBall(const WorldState& ws, MotionController* motion) {
  const bool ballInEdgeCase = checkBallOnLine(ws) || checkBallInPocket(ws);

  Vector2 target;
  double rotInput = 0;
  bool usePID;

  if (ws.peerRunning && ws.globalX < FieldConfig::HARD_BARRIER) {
    target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::HARD_BARRIER + GeneralConfig::BOT_DIAMETER / 2.0,
                           ws.globalY);
    rotInput = ws.heading;
    usePID = false;
  }

  else if (ballInEdgeCase) {
    usePID = true;
    //target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);
    target = getBallPursuitVec(ws);

    if (std::abs(ws.heading) >= GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      if (ws.ballRot > 0.0) {
        rotInput = ws.heading + GeneralConfig::HEADING_HARD_LIMIT_DEG;
      }
      else {
        rotInput = ws.heading - GeneralConfig::HEADING_HARD_LIMIT_DEG;
      }
    }
    else if (std::abs(ws.ballRot) > GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      rotInput = ws.heading;
    }
    else {
      rotInput = ws.ballRot;
    }
  }
  else {
    // normal pursuit
    usePID = true;
    target = getBallPursuitVec(ws);
    rotInput = ws.heading;
  }

  constexpr int dribblerSpeed = 100;

  const auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID, ws);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, true);
}
