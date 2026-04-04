#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

struct Kick {
  static void execute(const WorldState& ws, const MotionController* motion);
};

void executeKick(const WorldState& ws, const MotionController* motion);

struct ActionKick {
  void (*pFuncExec)(const WorldState&, const MotionController*) = &executeKick;
};
