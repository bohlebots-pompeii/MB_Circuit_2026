#pragma once

#include <nodes/ActionTypes.h>
#include <util/Vector2.hpp>

struct WorldState;
class MotionController;

Vector2 getHalfCircleTarget(const WorldState& ws);

void executeHalfCircleGuard(const WorldState& ws, MotionController* motion);

struct ActionHalfCircleGuard {
  ACTION_EXEC_FUNC pFuncExec = &executeHalfCircleGuard;
};
