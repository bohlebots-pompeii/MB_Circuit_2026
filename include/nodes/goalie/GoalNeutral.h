#pragma once
struct WorldState;
class MotionController;

namespace GoalNeutral {
  void execute(const WorldState& ws, MotionController* motion);
}
