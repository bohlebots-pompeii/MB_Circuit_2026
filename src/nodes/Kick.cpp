#include <nodes/Kick.h>
#include <motor_mb.h>

void Kick::execute(const WorldState& ws, const MotionController* motion) {
  (void)ws;
  (void)motion;
  setKick(true);
}

void executeKick(const WorldState& ws, const MotionController* motion) {
  Kick::execute(ws, motion);
}
