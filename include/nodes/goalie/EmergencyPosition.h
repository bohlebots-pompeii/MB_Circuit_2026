#pragma once
struct WorldState;
class MotionController;

namespace EmergencyPosition {
  void execute(const WorldState& ws, MotionController* motion);
}
