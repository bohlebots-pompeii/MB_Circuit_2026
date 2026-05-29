//
// Created by julius on 02.04.2026.
//

#include <MotionController.h>
#include <cmath>
#include "nodes/PassBetween.h"
#include "motor_mb.h"
#include "WorldState.h"
#include "config/config.h"
#include "util/Vector2.hpp"
#include "util/helper.h"

static double calculateShotAngle(const WorldState& ws, const Vector2& targetPos) {
  const auto position = Vector2(ws.globalX, ws.globalY);
  Serial.print("(");
  Serial.print(position.getX());
  Serial.print(", ");
  Serial.print(position.getY());
  Serial.print(") ");

  Serial.print("(");
  Serial.print(targetPos.getX());
  Serial.print(", ");
  Serial.print(targetPos.getY());
  Serial.print(") ");

  Vector2 dir = targetPos - position;
  dir.rotate(toRad(ws.heading));

  const double angleRad = atan2(dir.getY(), dir.getX());
  const double angle = toDeg(angleRad);
  Serial.println(angle);

  return angle;
}

void executePassBetween(const WorldState& ws, MotionController* motion) {
  if (!ws.peerAlive) {
    pushData(false, false, 0, 0, 0, 0, false);
    return;
  }
  const auto targetPosition = Vector2(ws.peerGlobalX, ws.peerGlobalY);
  const double angleSetpoint = calculateShotAngle(ws, targetPosition);
  const double a = ws.globalX - -60;
  const double b = ws.globalY - 0;
  double rotInput = 0;
  if (std::hypot(a, b) < FieldConfig::POINT_REACHED_DIST) {
    rotInput = angleSetpoint;
  }
  else {
    rotInput = ws.heading;
  }

  const auto target = getToPointVec(ws.globalX, ws.globalY, -60, 0);
  //const auto target = Vector2(0,0);

  auto [vx, vy, rot] = motion->compute(target, rotInput, true, ws);
  pushData(ws.ena, false, static_cast<int>(round(vx)), static_cast<int>(round(vy)), rot, 0, true);
}
