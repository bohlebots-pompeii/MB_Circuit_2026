#pragma once
#include <util/Vector2.hpp>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct DriveToNeutral {
  void execute(const WorldState& ws, MotionController* motion);

private:
  Vector2 getMoveToCenterVec(const WorldState& ws) const;
};

void executeDriveToNeutral(const WorldState& ws, MotionController* motion);

struct ActionDriveToNeutral {
  ACTION_EXEC_FUNC pFuncExec = &executeDriveToNeutral;
};
