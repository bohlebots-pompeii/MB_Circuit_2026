#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeDribbleToGoal(const WorldState& ws, MotionController* motion);

struct ActionDribbleToGoal {
  ACTION_EXEC_FUNC pFuncExec = &executeDribbleToGoal;
};
