#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct HiddenBallNPocket {
  void execute(const WorldState& ws, MotionController* motion);

private:
  [[nodiscard]] bool checkBallInPocket(const WorldState& ws) const;
};

void executeHiddenBallNPocket(const WorldState& ws, MotionController* motion);

struct ActionHiddenBallNPocket {
  ACTION_EXEC_FUNC pFuncExec = &executeHiddenBallNPocket;
};
