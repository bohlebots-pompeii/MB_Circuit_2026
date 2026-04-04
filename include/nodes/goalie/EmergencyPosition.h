#pragma once
#include <util/Vector2.hpp>
#include <util/MovingAverage.h>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct EmergencyPosition {
  void execute(const WorldState& ws, MotionController* motion);

private:
  MovingAverage<double, 10> emergencyBallAvgX;
  MovingAverage<double, 10> emergencyBallAvgY;
  Vector2 getEmergencyBallVec(const WorldState& ws);
  Vector2 getHalfCircleTarget(const WorldState& ws, const Vector2& ballVec) const;
};

void executeEmergencyPosition(const WorldState& ws, MotionController* motion);

struct ActionEmergencyPosition {
  ACTION_EXEC_FUNC pFuncExec = &executeEmergencyPosition;
};
