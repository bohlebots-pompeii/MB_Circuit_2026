#pragma once
struct WorldState;
class MotionController;

namespace DriveToNeutral {
  void execute(const WorldState& ws, MotionController* motion);
}
