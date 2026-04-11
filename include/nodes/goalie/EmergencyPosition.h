#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeEmergencyPosition(const WorldState& ws, MotionController* motion);

struct ActionEmergencyPosition {
  ACTION_EXEC_FUNC pFuncExec = &executeEmergencyPosition;
};
