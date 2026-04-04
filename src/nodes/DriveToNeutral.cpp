//
//  created by Julius on 02.04.2026
//

#include <nodes/DriveToNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <util/Vector2.hpp>

static Vector2 getMoveToCenterVec(const WorldState& ws) {
  if (!ws.peerAlive || !ws.peerRunning) {
    return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
  }

  return {-ws.globalX, -ws.globalY};
}

void executeDriveToNeutral(const WorldState& ws, MotionController* motion) {
  const Vector2 target = getMoveToCenterVec(ws);
  const double rotInput = ws.heading;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), true);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
}
