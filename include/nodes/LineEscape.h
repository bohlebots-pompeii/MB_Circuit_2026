#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeLineEscape(const WorldState& ws, MotionController* motion);

struct ActionLineEscape {
  ACTION_EXEC_FUNC pFuncExec = &executeLineEscape;
};
