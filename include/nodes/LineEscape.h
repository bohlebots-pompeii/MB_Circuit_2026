#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeLineEscape(const WorldState& ws, MotionController* motion);

bool checkInOwnPocket(const WorldState& ws);
bool checkInPocket(const WorldState& ws);
int checkOnLine(const WorldState& ws); // 0 no, 1 right, -1 left

struct ActionLineEscape {
  ACTION_EXEC_FUNC pFuncExec = &executeLineEscape;
};
