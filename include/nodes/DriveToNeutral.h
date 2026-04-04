#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeDriveToNeutral(const WorldState& ws, MotionController* motion);

struct ActionDriveToNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeDriveToNeutral;
};
