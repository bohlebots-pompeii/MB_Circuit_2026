//
//  created by Julius on 02.04.2026
//

#include <nodes/DriveToNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <algorithm>

DriveToNeutral::DriveToNeutral(std::shared_ptr<MotionController> motion) : BehaviorNode("SearchMode"),
                                                                           _motion(std::move(motion)) {}

// Purpose: Node updater
// Output: Status for the Node caller
// Input: the WorldState struct
BT::Status DriveToNeutral::tick(const WorldState& ws) {
  // Always running as fallback
  const Vector2 target = getMoveToCenterVec(ws);
  const double rotInput = ws.heading;

  auto [vx, vy, rot] = _motion->compute(target, static_cast<float>(rotInput), true);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);

  return BT::Status::RUNNING;
}

// Output: returns a vector that points either to the center (if the other bot is running) or else to the neutral point
// in front of the own goal.
// Input: the world state for the middle vector
// Assuming: the robots position on the field is valid
Vector2 DriveToNeutral::getMoveToCenterVec(const WorldState& ws) {
  if (!ws.peerAlive || !ws.peerRunning) {
    return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
  }

  return Vector2(-ws.globalX, -ws.globalY);
}