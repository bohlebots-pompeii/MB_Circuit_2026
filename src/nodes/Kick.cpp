#include <nodes/Kick.h>
#include <motor_mb.h>

void executeKick(const WorldState& ws, const MotionController* motion) {
  (void)ws;
  (void)motion;
  setKick(true);
}
