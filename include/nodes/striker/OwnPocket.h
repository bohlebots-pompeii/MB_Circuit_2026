#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

bool checkBallInOwnPocket(const WorldState& ws);

void executeOwnPocket(const WorldState& ws, MotionController* motion);

struct ActionOwnPocket {
  ACTION_EXEC_FUNC pFuncExec = &executeOwnPocket;
};
