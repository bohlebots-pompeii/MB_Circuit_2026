#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

namespace PassBetween {
  void execute(const WorldState& ws, MotionController* motion);
}

struct ActionPassBetween {
  ACTION_EXEC_FUNC pFuncExec = &PassBetween::execute;
};
