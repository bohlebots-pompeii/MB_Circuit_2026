#pragma once

#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeHalfCircleGuard(const WorldState& ws, MotionController* motion);

struct ActionHalfCircleGuard {
  ACTION_EXEC_FUNC pFuncExec = &executeHalfCircleGuard;
};
