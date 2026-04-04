#pragma once
#include <util/Vector2.hpp>
#include <elapsedMillis.h>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct InterceptBall {
  void execute(const WorldState& ws, MotionController* motion);
  bool isDrivingToBall(const WorldState& ws);

private:
  elapsedMillis ballMovementTimer;
  elapsedMillis drivingToBallTimer;
  bool drivingToBall = false;
  Vector2 lastBallVec{0, 0};
  void updateTimers(const WorldState& ws);
};

void executeInterceptBall(const WorldState& ws, MotionController* motion);
bool canExecuteInterceptBall(const WorldState& ws);

struct ActionInterceptBall {
  ACTION_EXEC_FUNC pFuncExec = &executeInterceptBall;
};
