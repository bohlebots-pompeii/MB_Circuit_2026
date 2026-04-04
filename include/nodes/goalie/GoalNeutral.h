#pragma once

#include <nodes/ActionTypes.h>

struct GoalNeutral {
  void execute(const WorldState& ws, MotionController* motion);
};

void executeGoalNeutral(const WorldState& ws, MotionController* motion);

struct ActionGoalNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeGoalNeutral;
};
