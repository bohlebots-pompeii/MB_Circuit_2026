#pragma once
struct WorldState;
class MotionController;

namespace HoldNeutral {
  void execute(const WorldState& ws, MotionController* motion);
}
