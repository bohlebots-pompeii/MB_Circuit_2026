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

  double ballGlobalX = ws.globalX + std::cos(ballRadians) * ws.ballDist;

  bool ballInOwnPocket = false;

  if (ballGlobalX < (-FieldConfig::Hx) + 20.0) ballInOwnPocket = true;

  return ballInOwnPocket;
}

void executeOwnPocket(const WorldState& ws, MotionController* motion){
  constexpr bool useRotDelta = true;
  const auto rotInput = static_cast<float>(ws.ownGoalRot - 180.0);
  Vector2 target = getHalfCircleTarget(ws);

  if (abs(ws.ballRot) < 10.0) {
    target = ws.ballVec;
  }

  auto [vx, vy, rot] = motion->compute(target, rotInput, true);

  constexpr int dribblerSpeed = 100;
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, useRotDelta);
}
