#pragma once
#include <util/Vector2.hpp>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct DribbleToGoal {
  void execute(const WorldState& ws, MotionController* motion);

private:
  Vector2 getBallAlignedVec(const WorldState& ws, int speed);
};

void executeDribbleToGoal(const WorldState& ws, MotionController* motion);

struct ActionDribbleToGoal {
  ACTION_EXEC_FUNC pFuncExec = &executeDribbleToGoal;
};
