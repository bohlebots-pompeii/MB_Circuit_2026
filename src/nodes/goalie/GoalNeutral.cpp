#include <nodes/goalie/GoalNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>

void executeGoalNeutral(const WorldState& ws, MotionController* motion) {
  Vector2 target;
  if (ws.lastBallSeenTime < 2500) {
    target = Vector2(0, 0);
  }
  else {
    target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GOAL_NEUTRAL_POS_X, 0);
  }
  const auto rotInput = static_cast<float>(ws.heading);
  constexpr bool usePID = true;

  auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
}
