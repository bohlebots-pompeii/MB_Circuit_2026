#pragma once
struct WorldState;
class MotionController;

namespace DribbleToGoal {
  void execute(const WorldState& ws, MotionController* motion);
}
