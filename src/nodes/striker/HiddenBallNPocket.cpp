//
// Created by julius on 17.03.2026.
//

#include <nodes/striker/HiddenBallNPocket.h>
#include <MotionController.h>
#include <WorldState.h>
#include <config/config.h>
#include <motor_mb.h>
#include <util/Vector2.hpp>
#include <cmath>

static bool checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  return std::abs(absoluteGoalDir) > FieldConfig::IN_POCKET_ANGLE;
}

void executeHiddenBallNPocket(const WorldState& ws, MotionController* motion) {
  double rotIn = 0.0;
  Vector2 target(0, 0);

  const bool inPocket = checkBallInPocket(ws);
  const bool closeToGoal = ws.targetGoalDist < FieldConfig::GX - GeneralConfig::BOT_DIAMETER;
  const bool farFromGoal = ws.targetGoalDist > FieldConfig::GX - GeneralConfig::BOT_DIAMETER * 1.5;

  if (!GeneralConfig::USE_HIDDEN_BALL) {
    if (inPocket) {
      rotIn = -ws.awayFromOwnGoalAngle / 5.0;
      target = ws.ownGoalVec;
      target.setY(0 - ws.globalY * 3);
    }

    if (target.getMagnitude() > 0.001) {
      target.normalize();
      target *= 15.0;
    }

    auto [vx, vy, rot] = motion->compute(target, 0.0f, false, ws);
    rot = static_cast<int>(std::round(rotIn));

    constexpr int dribblerSpeed = 100;
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, false);

    return;
  }

  if (!inPocket && closeToGoal) {
    rotIn = -(ws.targetGoalRot - 5.0) / 3.0;
  }
  else if (inPocket) {
    rotIn = -ws.awayFromOwnGoalAngle / 5.0;
    target = ws.ownGoalVec;
    target.setY(0 - ws.globalY * 3);
  }
  else if (farFromGoal && std::abs(ws.heading) > 40.0) {
    double awayFromTargetGoalRot = ws.targetGoalRot - 180.0;
    while (awayFromTargetGoalRot > 180.0) awayFromTargetGoalRot -= 360.0;
    while (awayFromTargetGoalRot < -180.0) awayFromTargetGoalRot += 360.0;

    rotIn = -awayFromTargetGoalRot / 5.0;
    target = ws.targetGoalVec;
  }
  else {
    rotIn = -ws.awayFromOwnGoalAngle / 5.0;
    target = ws.targetGoalVec;
  }

  if (rotIn > 15.0) rotIn = 15.0;
  if (rotIn < -15.0) rotIn = -15.0;

  if (target.getMagnitude() > 0.001) {
    target.normalize();
    target *= 15.0;
  }

  auto [vx, vy, rot] = motion->compute(target, 0.0f, false, ws);
  rot = static_cast<int>(std::round(rotIn));

  constexpr int dribblerSpeed = 100;
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, false);
}
