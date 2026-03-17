//
// Created by julius on 17.03.2026.
//

#include "RetrieveFromPocket.h"

bool RetriveFromPocket::checkBallInPocket(const WorldState& ws) const {
  const double ballX = ws.ballVec.getX();

  float globalGoalDir = ws.targetGoalRot - ws.heading;
  while (globalGoalDir > 180) globalGoalDir -= 360;
  while (globalGoalDir < -180) globalGoalDir += 360;

  if (!ws.ballExists && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
    return true;
  }

  if (ballX > 0 && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
    return true;
  }

  return false;
}