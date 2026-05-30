//
// Created by Timo on 27.05.26.
//

#include "nodes/striker/OwnPocket.h"
#include <MotionController.h>
#include <WorldState.h>
#include <config/config.h>
#include <motor_mb.h>
#include <util/Vector2.hpp>
#include <cmath>
#include <nodes/goalie/HalfCircleGuard.h>

bool checkBotInOwnPocket(const WorldState& ws) {
  double globalOwnGoalRot = ws.ownGoalRot - ws.heading;
  if (globalOwnGoalRot > 180.0) globalOwnGoalRot -= 360.0;
  if (globalOwnGoalRot < -180.0) globalOwnGoalRot += 360.0;

  if (std::abs(globalOwnGoalRot) > FieldConfig::IN_POCKET_ANGLE) {
    return true;
  }
  return false;
}

bool checkBallInOwnPocket(const WorldState& ws) {
  if (!ws.ballExists) return false;

  Vector2 ballVec = ws.ballVec;
  ballVec.rotate(toRad(-ws.heading));

  const Vector2 ballGlobal = Vector2(ws.globalX, ws.globalY) + ballVec;

  if (ballGlobal.getX() < 0 &&
    std::abs(ballGlobal.getX()) > FieldConfig::GX &&
    std::abs(ballGlobal.getY()) > FieldConfig::GY &&
    checkBotInOwnPocket(ws)) {
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
    rotInput = ws.ballRot / 2.0;
  }

  if (ws.peerRunning && ws.peerAlive) {
    if (ws.globalX < FieldConfig::HARD_BARRIER) {
      target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::HARD_BARRIER, 0);
      rotInput = ws.heading;
    }
  }

  const int drib = ws.ballDist < 40.0 && ws.ballDist != 0.0 ? 100 : 0;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID, ws);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
