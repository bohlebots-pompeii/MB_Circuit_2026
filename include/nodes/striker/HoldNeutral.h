#pragma once

#include <util/Vector2.hpp>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct HoldNeutral {
  void execute(const WorldState& ws, MotionController* motion);

private:
  Vector2 _lastDriveVec{0, 0};
};

void executeHoldNeutral(const WorldState& ws, MotionController* motion);

struct ActionHoldNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeHoldNeutral;
};
