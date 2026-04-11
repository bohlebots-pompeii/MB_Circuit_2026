#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeHiddenBallNPocket(const WorldState& ws, MotionController* motion);

struct ActionHiddenBallNPocket {
  ACTION_EXEC_FUNC pFuncExec = &executeHiddenBallNPocket;
};
