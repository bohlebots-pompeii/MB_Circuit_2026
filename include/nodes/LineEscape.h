#pragma once
#include <util/Vector2.hpp>
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct LineEscape {
  void execute(const WorldState& ws, MotionController* motion);

private:
  [[nodiscard]] Vector2 getAwayFromLineVec(const WorldState& ws, int speed) const;
  [[nodiscard]] bool checkBallOnLine(const WorldState& ws) const;
};

void executeLineEscape(const WorldState& ws, MotionController* motion);

struct ActionLineEscape {
  ACTION_EXEC_FUNC pFuncExec = &executeLineEscape;
};
