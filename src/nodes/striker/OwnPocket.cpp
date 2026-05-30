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

  Vector2 ballVec = ws.ballVec;
  ballVec.rotate(toRad(-ws.heading));

  const Vector2 ballGlobal = Vector2(ws.globalX, ws.globalY) + ballVec;

  if (ballGlobal.getX() < 0 &&
    std::abs(ballGlobal.getX()) > FieldConfig::Hx - 50.0 &&
    std::abs(ballGlobal.getY()) > FieldConfig::Wy - 50.0) {
    return true;
  }

  return false;
}

void executeOwnPocket(const WorldState& ws, MotionController* motion, const ActionHalfCircleGuard& halfCircleGuard) {
  auto rotInput = ws.awayFromOwnGoalAngle;
  bool usePID = true;

  Vector2 target = halfCircleGuard.pFuncGetHalfCircleTarget(ws);

  // if ball Close enough
  if (std::abs(ws.ballRot) < 20.0) {
    target = ws.ballVec;
    target.normalize();
    target *= 20.0;
    usePID = false;
  }

  // check if we're not looking at the goal
  if (std::abs(ws.ownGoalRot) < 150.0) {
    rotInput = ws.ballRot;
  }

  const int drib = ws.ballDist < 40.0 && ws.ballDist != 0.0 ? 100 : 0;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID, ws);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
