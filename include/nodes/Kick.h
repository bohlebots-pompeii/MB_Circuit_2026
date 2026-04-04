#pragma once
struct WorldState;
class MotionController;

namespace Kick {
  void execute(const WorldState& ws, MotionController* motion);
}
