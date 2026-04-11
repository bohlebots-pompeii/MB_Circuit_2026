#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeGetBehindBall(const WorldState& ws, MotionController* motion);

struct ActionGetBehindBall {
  ACTION_EXEC_FUNC pFuncExec = &executeGetBehindBall;
};
