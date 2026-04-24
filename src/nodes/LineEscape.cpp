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

  auto midVec = Vector2(-ws.globalX, -ws.globalY);
  midVec.normalize();
  midVec.rotate(ws.heading);

  Vector2 target = line * 0.0 + midVec * 1.0;
  target.normalize();
  target *= speed;

  return target;
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
  Serial.println(target.getX());
  Serial.println(target.getY());
  float rotInput = 0;

  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  if (checkBallOnLine(ws)) {
    if (ws.heading < FieldConfig::rotateToBallAngle) {
      rotInput = static_cast<float>(ws.ballRot);
    }
    else {
      rotInput = ws.heading - std::copysign(ws.ballRot, FieldConfig::rotateToBallAngle);
    }
  }
  else {
    rotInput = ws.heading;
  }

  auto [vx, vy, rot] = motion->compute(target, rotInput, false);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 100, true);
}
