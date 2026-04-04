#pragma once

#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeGoalNeutral(const WorldState& ws, MotionController* motion);

struct ActionGoalNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeGoalNeutral;
};
