#pragma once
#include <nodes/ActionTypes.h>

struct WorldState;
class MotionController;

void executeKick(const WorldState& ws, const MotionController* motion);

struct ActionKick {
  void (*pFuncExec)(const WorldState&, const MotionController*) = &executeKick;
};
