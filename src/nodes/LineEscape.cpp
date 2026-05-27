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

  Vector2 target;
  if (ws.isGoalie) {
    target = line * 0.7 + midVec * 0.3;
    if (checkInOwnPocket(ws)) {
      if (target.getX() < 0.0) {
        target.setX(-target.getX());
      }
    }
  }
  else {
    target = midVec;
    if (checkInPocket(ws)) {
      target = line * 0.7 + midVec * 0.3;
    }
  }

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

  if (globalY > FieldConfig::LINE_POS_Y && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -FieldConfig::LINE_POS_Y && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

bool checkInOwnPocket(const WorldState& ws) {
  double globalOwnGoalRot = ws.ownGoalRot - ws.heading;
  if (globalOwnGoalRot > 180.0) globalOwnGoalRot -= 360.0;
  if (globalOwnGoalRot < -180.0) globalOwnGoalRot += 360.0;

  if (std::abs(globalOwnGoalRot) > FieldConfig::IN_POCKET_ANGLE) {
    return true;
  }
  return false;
}

bool checkInPocket(const WorldState& ws) {
  double globalGoalRot = ws.targetGoalRot - ws.heading;
  if (globalGoalRot > 180.0) globalGoalRot -= 360.0;
  if (globalGoalRot < -180.0) globalGoalRot += 360.0;

  if (std::abs(globalGoalRot) > FieldConfig::IN_POCKET_ANGLE) {
    return true;
  }
  return false;
}

int checkOnLine(const WorldState& ws) {
  if (std::abs(ws.globalY) > FieldConfig::LINE_POS_Y) {
    if (ws.globalY > 0.0) return 1;
    return -1;
  }
  return 0;
}

void executeLineEscape(const WorldState& ws, MotionController* motion) {
  const Vector2 target = getAwayFromLineVec(ws, 30);
  float rotInput = 0;

  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  if (checkBallOnLine(ws)) {
    if (std::abs(ws.heading) >= GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      if (ws.ballRot > 0.0) {
        rotInput = static_cast<float>(ws.heading + GeneralConfig::HEADING_HARD_LIMIT_DEG);
      }
      else {
        rotInput = static_cast<float>(ws.heading - GeneralConfig::HEADING_HARD_LIMIT_DEG);
      }
    }
    else if (std::abs(ws.ballRot) > GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      rotInput = static_cast<float>(ws.heading);
    }
    else {
      rotInput = static_cast<float>(ws.ballRot);
    }
  }
  else {
    rotInput = static_cast<float>(ws.heading);
  }

  auto [vx, vy, rot] = motion->compute(target, rotInput, false);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 100, true);
}
