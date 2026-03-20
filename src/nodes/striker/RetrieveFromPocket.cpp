//
// Created by julius on 17.03.2026.
//

#include "nodes/striker/RetrieveFromPocket.h"
#include <config/config.h>
#include <WorldState.h>

#include "MotionController.h"
#include "motor_mb.h"

RetrieveFromPocket::RetrieveFromPocket(std::shared_ptr<MotionController> motion)
  : _motion(std::move(motion)) {
  hasBallTimer = 0;
}

BT::Status RetrieveFromPocket::tick(const WorldState& ws) {
  if (!ws.hasBall) {
    hasBallTimer = 0;
    return BT::Status::FAILURE;
  }

  if (hasBallTimer < GeneralConfig::HasBallValidTime) {
    return BT::Status::FAILURE;
  }

  int rot = 0;
  Vector2 target;

  double globalGoalDir = ws.targetGoalRot - ws.heading;
  while (globalGoalDir > 180) globalGoalDir -= 360;
  while (globalGoalDir < -180) globalGoalDir += 360;

  if (checkBallInPocket(ws)) {
    target = Vector2(-1, 0);
    target *= 15;
    rot = 0;
  }
  else if (std::abs(globalGoalDir) < FieldConfig::FieldPocketAngle && std::abs(globalGoalDir) >
    FieldConfig::FieldPocketAngle - 10.0) {
    target = Vector2(0, 0);
    rot = ws.targetGoalRot > 0 ? -15 : 15;
  }
  else {
    return BT::Status::FAILURE;
  }

  MotionController::Output out = _motion->compute(target, 0, false);
  out.rot = rot;

  constexpr int dribblerSpeed = 100;

  pushData(ws.ena, false, static_cast<int>(out.vx), static_cast<int>(out.vy), out.rot, dribblerSpeed, false);

  return BT::Status::RUNNING;
}

bool RetrieveFromPocket::checkBallInPocket(const WorldState& ws) {
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
