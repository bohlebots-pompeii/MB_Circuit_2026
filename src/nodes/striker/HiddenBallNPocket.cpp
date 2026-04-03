//
// Created by julius on 17.03.2026.
//

#include "nodes/striker/HiddenBallNPocket.h"
#include <config/config.h>
#include <WorldState.h>

#include "MotionController.h"
#include "motor_mb.h"

HiddenBallNPocket::HiddenBallNPocket(std::shared_ptr<MotionController> motion)
  : BehaviorNode("RetrieveFromPocket"), _motion(std::move(motion)) {}

BT::Status HiddenBallNPocket::tick(const WorldState& ws) {
  if (!ws.hasBall) {
    return BT::Status::FAILURE;
  }

  if (ws.hasBallTime < GeneralConfig::HasBallValidTime) {
    return BT::Status::FAILURE;
  }

  double rotIn = 0;
  Vector2 target;

  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  auto clampRot = [](const double r) {
    return constrain(r, -15.0, 15.0);
  };

  const bool inPocket = checkBallInPocket(ws);
  const bool closeToGoal = ws.targetGoalDist < FieldConfig::kickDistance - 4.0;
  const bool farFromGoal = ws.targetGoalDist > FieldConfig::kickDistance + 20.0;

  if (!inPocket && closeToGoal) {
    rotIn = clampRot(-(ws.targetGoalRot - 5.0) / 3.0);
    target = Vector2(0, 0);
  }

  else if (inPocket) {
    rotIn = clampRot(-ws.awayFromOwnGoalAngle / 5.0);
    target = ws.ownGoalVec;
    target.setY(target.getY() * 4.0);
  }

  // hidden ball tech
  else if (farFromGoal) {
    double awayFromTargetGoalRot = ws.targetGoalRot - 180.0;
    while (awayFromTargetGoalRot > 180.0) awayFromTargetGoalRot -= 360.0;
    while (awayFromTargetGoalRot < -180.0) awayFromTargetGoalRot += 360.0;

    rotIn = clampRot(-awayFromTargetGoalRot / 5.0);
    target = ws.targetGoalVec;
  }

  // buffer zone
  else {
    rotIn = clampRot(-ws.awayFromOwnGoalAngle / 5.0);
    target = ws.targetGoalVec;
  }

  if (target.getMagnitude() > 0.001) {
    target.normalize();
    target *= 15.0;
  }

  auto [vx, vy, rot] = _motion->compute(target, 0, false);
  rot = static_cast<int>(std::round(rotIn));

  constexpr int dribblerSpeed = 100;

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, false);

  return BT::Status::RUNNING;
}

bool HiddenBallNPocket::checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  if (std::abs(absoluteGoalDir) > FieldConfig::FieldPocketAngle) {
    return true;
  }

  return false;
}
