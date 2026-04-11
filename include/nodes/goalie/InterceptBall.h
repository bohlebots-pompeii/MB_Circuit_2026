#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeInterceptBall(const WorldState& ws, MotionController* motion);
bool canExecuteInterceptBall(const WorldState& ws);

struct ActionInterceptBall {
  ACTION_EXEC_FUNC pFuncExec = &executeInterceptBall;
};
