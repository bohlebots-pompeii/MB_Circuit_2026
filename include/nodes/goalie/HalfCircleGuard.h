#pragma once
struct WorldState;
class MotionController;

namespace HalfCircleGuard {
  void execute(const WorldState& ws, MotionController* motion);
}
