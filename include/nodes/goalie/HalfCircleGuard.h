#pragma once

#include <nodes/ActionTypes.h>
#include <util/Vector2.hpp>

struct WorldState;
class MotionController;

void executeHalfCircleGuard(const WorldState& ws, MotionController* motion);

Vector2 getHalfCircleTarget(const WorldState& ws);

Vector2 driveOnLine(const WorldState& ws, const Vector2& target);

Vector2 getAwayFromLineVec(const WorldState& ws);

void applyBallAvoidance(const WorldState& ws, Vector2& target);

struct ActionHalfCircleGuard {
  ACTION_EXEC_FUNC pFuncExec = &executeHalfCircleGuard;
};
