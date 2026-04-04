#include <nodes/striker/HoldNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>

void HoldNeutral::execute(const WorldState& ws, MotionController* motion) {
  if (ws.lastBallSeenTime < 30) {
    _lastDriveVec = motion->getLastTarget();
  }

  if (_lastDriveVec.getMagnitude() < 1e-3) {
    _lastDriveVec = Vector2(0.0, 0.0);
  }

  constexpr float rotInput = 0.0f;
  constexpr bool usePID = true;
  constexpr int dribSpeed = 100;

  auto [vx, vy, rot] = motion->compute(_lastDriveVec, rotInput, usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribSpeed, true);
}

void executeHoldNeutral(const WorldState& ws, MotionController* motion) {
  static HoldNeutral action;
  action.execute(ws, motion);
}
