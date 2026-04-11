#include <nodes/LineEscape.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <config/config.h>
#include <cmath>
#include <numbers>

static Vector2 getAwayFromLineVec(const WorldState& ws, int speed) {
  Vector2 line = degToVec(ws.lineRot);
  line.rotate(std::numbers::pi);

  Vector2 midVec(-ws.globalX, -ws.globalY);
  if (midVec.getMagnitude() > 1e-3) {
    midVec.normalize();
  }

  line = line * 0.3f + midVec * 0.7f;
  line.normalize();

  return line * speed;
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

void executeLineEscape(const WorldState& ws, MotionController* motion) {
  const Vector2 target = getAwayFromLineVec(ws, 30);
  float rotInput = 0;

  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  if (checkBallOnLine(ws)) {
    if (std::abs(globalBallDir) < FieldConfig::rotateToBallAngle) {
      rotInput = static_cast<float>(ws.ballRot);
    }
    else {
      rotInput = 0.0;
    }
  }
  else {
    rotInput = 0.0;
  }

  auto [vx, vy, rot] = motion->compute(target, rotInput, false);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 100, true);
}
