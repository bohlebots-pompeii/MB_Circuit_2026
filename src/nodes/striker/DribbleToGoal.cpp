#include <nodes/striker/DribbleToGoal.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>

#include "config/config.h"

static Vector2 getBallAlignedVec(const WorldState& ws, const int speed) {
  Vector2 target = degToVec(ws.targetGoalTargetRot);
  target.normalize();
  return target * speed;
}

void executeDribbleToGoal(const WorldState& ws, MotionController* motion) {
  constexpr bool useRotDelta = false;
  const auto rotInput = ws.targetGoalTargetRot / 2.0;
  const Vector2 target = getBallAlignedVec(ws, 70);

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), false, ws);

  int dribblerSpeed = 100;
  if (std::abs(ws.targetGoalTargetRot) < 15.0) {
    dribblerSpeed = 15;
  }
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, useRotDelta);
}
