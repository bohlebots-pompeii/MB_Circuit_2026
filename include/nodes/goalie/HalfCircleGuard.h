#pragma once

#include <util/Vector2.hpp>
#include <util/MovingAverage.h>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct HalfCircleGuard {
  void execute(const WorldState& ws, MotionController* motion);

private:
  MovingAverage<double, 10> strikerAvgX;
  MovingAverage<double, 10> strikerAvgY;
  Vector2 getHalfCircleTarget(const WorldState& ws) const;
  Vector2 getAwayFromLineVec(const WorldState& ws) const;
  Vector2 driveOnLine(const WorldState& ws, const Vector2& target) const;
  void applyBallAvoidance(const WorldState& ws, Vector2& target) const;
  void applyStrikerAvoidance(const WorldState& ws, Vector2& target);
};

void executeHalfCircleGuard(const WorldState& ws, MotionController* motion);

struct ActionHalfCircleGuard {
  ACTION_EXEC_FUNC pFuncExec = &executeHalfCircleGuard;
};
