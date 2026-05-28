//
// Created by Timo on 27.05.26.
//

#include "../../../include/nodes/striker/OwnPocket.h"
#include <MotionController.h>
#include <WorldState.h>
#include <config/config.h>
#include <motor_mb.h>
#include <util/Vector2.hpp>
#include <cmath>
#include <nodes/goalie/HalfCircleGuard.h>

bool checkBallInOwnPocket(const WorldState& ws) {
  if (!ws.ballExists) return false;

  double globalBallRot = ws.ballRot - ws.heading;
  if (globalBallRot > 180.0) globalBallRot -= 360.0;
  if (globalBallRot < -180.0) globalBallRot += 360.0;

  const double ballRadians = globalBallRot * (std::numbers::pi / 180.0);

  const double ballGlobalX = ws.globalX + std::cos(ballRadians) * ws.ballDist;

  bool ballInOwnPocket = false;

  if (ballGlobalX < (-FieldConfig::Hx) + 25.0) ballInOwnPocket = true;

  return ballInOwnPocket;
}

void executeOwnPocket(const WorldState& ws, MotionController* motion){
  Vector2 target;
  auto rotInput = ws.awayFromOwnGoalAngle;
  constexpr bool usePID = true;

  if (ws.lineSeen) {
    if (ws.lineProgress < 16) {
      const Vector2 desiredTarget = getHalfCircleTarget(ws);
      target = driveOnLine(ws, desiredTarget);
    }
    else {
      target = getAwayFromLineVec(ws);
    }
  }
  else {
    target = getHalfCircleTarget(ws);
  }

  const int drib = ws.ballDist < 40 && ws.ballDist != 0 ? 100 : 0;

  if (abs(ws.ownGoalRot) < 150) {
    rotInput = ws.ballRot;
    target = Vector2(target.getX() / 3, target.getY() / 3);
  }

  if (ws.globalY < 0) {
    rotInput -= 20;
  }
  else {
    rotInput += 20;
  }

  if (abs(ws.ballRot) < 10.0) {
    target = ws.ballVec;
  }

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
