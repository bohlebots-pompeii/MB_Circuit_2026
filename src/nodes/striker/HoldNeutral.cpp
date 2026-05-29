#include <nodes/striker/HoldNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/Vector2.hpp>

static Vector2 g_lastDriveVec{0, 0};

void executeHoldNeutral(const WorldState& ws, MotionController* motion) {
  if (ws.lastBallSeenTime < 30) {
    g_lastDriveVec = motion->getLastTarget();
  }

  if (g_lastDriveVec.getMagnitude() < 1e-3) {
    g_lastDriveVec = Vector2(0.0, 0.0);
  }

  constexpr float rotInput = 0.0f;
  constexpr bool usePID = true;
  constexpr int dribSpeed = 100;

  auto [vx, vy, rot] = motion->compute(g_lastDriveVec, rotInput, usePID, ws);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribSpeed, true);
}
