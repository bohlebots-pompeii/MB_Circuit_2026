#pragma once
struct WorldState;
class MotionController;

namespace PassBetween {
  void execute(const WorldState& ws, MotionController* motion);
}
