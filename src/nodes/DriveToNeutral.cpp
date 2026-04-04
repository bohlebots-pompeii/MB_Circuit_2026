//
//  created by Julius on 02.04.2026
//

#include <MotionController.h>
#include <nodes/DriveToNeutral.h>
#include <WorldState.h>
#include <motor_mb.h>
#include <config/config.h>

namespace DriveToNeutral {
  Vector2 getMoveToCenterVec(const WorldState& ws);
  // Purpose: Node updater
  // Output: Status for the Node caller
  // Input: the WorldState struct
  void execute(const WorldState& ws, MotionController* motion) {
    // Always running as fallback
    const Vector2 target = getMoveToCenterVec(ws);
    const double rotInput = ws.heading;

    auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), true);

    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
  }

  // Output: returns a vector that points either to the center (if the other bot is running) or else to the neutral point
  // in front of the own goal.
  // Input: the world state for the middle vector
  // Assuming: the robots position on the field is valid
  Vector2 getMoveToCenterVec(const WorldState& ws) {
    if (!ws.peerAlive || !ws.peerRunning) {
      return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    }

    return Vector2(-ws.globalX, -ws.globalY);
  }
}
