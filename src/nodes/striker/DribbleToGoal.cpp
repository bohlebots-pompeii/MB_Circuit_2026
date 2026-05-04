#include <nodes/striker/DribbleToGoal.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>

static Vector2 getBallAlignedVec(const WorldState& ws, int speed) {
  Vector2 target = degToVec(ws.targetGoalRot);
  target.normalize();
  return target * speed;
}

void executeDribbleToGoal(const WorldState& ws, MotionController* motion) {
  constexpr bool useRotDelta = true;
  const auto rotInput = static_cast<float>(ws.targetGoalRot);
  const Vector2 target = getBallAlignedVec(ws, 50);

  auto [vx, vy, rot] = motion->compute(target, rotInput, true);

  constexpr int dribblerSpeed = 100;
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, useRotDelta);
}
