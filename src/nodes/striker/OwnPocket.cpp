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

  if (ballGlobalX < (-FieldConfig::Hx) + 50.0) ballInOwnPocket = true;

  return ballInOwnPocket;
}

void executeOwnPocket(const WorldState& ws, MotionController* motion, const ActionHalfCircleGuard& halfCircleGuard){
  Vector2 target;
  auto rotInput = ws.awayFromOwnGoalAngle;
  constexpr bool usePID = true;

  if (ws.lineSeen) {
    if (ws.lineProgress < 16) {
      const Vector2 desiredTarget = halfCircleGuard.pFuncGetHalfCircleTarget(ws);
      target = halfCircleGuard.pFuncDriveOnLine(ws, desiredTarget);
    }
    else {
      target = halfCircleGuard.pFuncGetAwayFromLineVec(ws);
    }
  }
  else {
    target = halfCircleGuard.pFuncGetHalfCircleTarget(ws);
  }

  if (std::abs(ws.ballRot) < 20) {
    target = ws.ballVec;
  }

  if (abs(ws.ownGoalRot) < 150) {
    rotInput = ws.ballRot;
  }

  const int drib = ws.ballDist < 40 && ws.ballDist != 0 ? 100 : 0;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
