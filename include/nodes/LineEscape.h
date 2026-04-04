#pragma once
struct WorldState;
class MotionController;

namespace LineEscape {
  void execute(const WorldState& ws, MotionController* motion);
}
