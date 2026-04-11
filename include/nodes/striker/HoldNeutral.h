#pragma once

#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeHoldNeutral(const WorldState& ws, MotionController* motion);

struct ActionHoldNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeHoldNeutral;
};
