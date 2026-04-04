#pragma once
struct WorldState;
class MotionController;

namespace InterceptBall {
  void execute(const WorldState& ws, MotionController* motion);
  bool isDrivingToBall(const WorldState& ws);
}
