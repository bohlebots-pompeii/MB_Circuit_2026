#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executePassBetween(const WorldState& ws, MotionController* motion);

struct ActionPassBetween {
  ACTION_EXEC_FUNC pFuncExec = &executePassBetween;
};
