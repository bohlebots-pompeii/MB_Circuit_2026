#pragma once
#include <util/Vector2.hpp>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct GetBehindBall {
  void execute(const WorldState& ws, MotionController* motion);

private:
  Vector2 getBallPursuitVec(const WorldState& ws) const;
  Vector2 getBallApproachVec(const WorldState& ws, int speed) const;
  Vector2 getBallAlignedVec(const WorldState& ws, int speed) const;
  bool checkBallOnLine(const WorldState& ws) const;
  bool checkBallInPocket(const WorldState& ws) const;
};

void executeGetBehindBall(const WorldState& ws, MotionController* motion);

struct ActionGetBehindBall {
  ACTION_EXEC_FUNC pFuncExec = &executeGetBehindBall;
};
