#pragma once
#include <nodes/ActionTypes.h>

#include "nodes/goalie/HalfCircleGuard.h"

struct WorldState;
class MotionController;

bool checkBallInOwnPocket(const WorldState& ws);

void executeOwnPocket(const WorldState& ws, MotionController* motion, const ActionHalfCircleGuard& halfCircleGuard);

struct ActionOwnPocket {
  ACTION_EXEC_FUNC_HCG pFuncExec = &executeOwnPocket;
  ACTION_CHECK_FUNC pFuncCheck = &checkBallInOwnPocket;
};
