//
// Created by julius on 02.04.2026.
//

#include <MotionController.h>
#include <cmath>
#include "nodes/PassBetween.h"
#include "motor_mb.h"
#include "WorldState.h"
#include "util/Vector2.hpp"
#include "util/helper.h"

namespace PassBetween {
  double calculateShotAngle(const WorldState& ws, const Vector2& targetPos);

  void execute(const WorldState& ws, MotionController* motion) {
    const double angleSetpoint = calculateShotAngle(ws, Vector2(0, 0));

    const double rotInput = angleSetpoint;

    const auto target = Vector2(0, 0);

    auto [vx, vy, rot] = motion->compute(target, rotInput);
    pushData(ws.ena, false, vx, vy, rot, 0, true);
  }

  // purpose: calculate the absolute angle the robot has to turn to, to shoot to preferred destinatiod // input:
  double calculateShotAngle(const WorldState& ws, const Vector2& targetPos) {
    const auto _position = Vector2(ws.globalX, ws.globalY);
    Serial.print("(");
    Serial.print(_position.getX());
    Serial.print(", ");
    Serial.print(_position.getY());
    Serial.print(") ");
    const auto _targetPos = targetPos;
    Serial.print("(");
    Serial.print(_targetPos.getX());
    Serial.print(", ");
    Serial.print(_targetPos.getY());
    Serial.print(") ");

    const Vector2 dir = _targetPos - _position;
    const double angleRad = atan2(dir.getY(), dir.getX());
    const double angle = toDeg(angleRad);
    Serial.println(angle);

    return 0;
  }
}
