#include <nodes/striker/DribbleToGoal.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <config/config.h>

void DribbleToGoal::execute(const WorldState& ws, MotionController* motion) {
  constexpr bool useRotDelta = true;
  float rotInput = 0;

  rotInput = static_cast<float>(ws.targetGoalRot) / 2;
  const Vector2 target = getBallAlignedVec(ws, 50);

  auto [vx, vy, rot] = motion->compute(target, rotInput, true);

  //const int dribblerSpeed = target.getX() > 10 ? 50 : 100;
  constexpr int dribblerSpeed = 100;

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, useRotDelta);
}

Vector2 DribbleToGoal::getBallAlignedVec(const WorldState& ws, int speed) {
  Vector2 target = degToVec(ws.targetGoalRot);
  target.normalize();
  return target * speed;
}

void executeDribbleToGoal(const WorldState& ws, MotionController* motion) {
  static DribbleToGoal action;
  action.execute(ws, motion);
}
