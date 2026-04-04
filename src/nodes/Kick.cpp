#include <nodes/Kick.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <cmath>
#include <config/config.h>

namespace Kick {
  // decider if the bot should kick or not
  void execute(const WorldState& ws, MotionController* motion) {
    setKick(false);

    if (!ws.hasBall) {
      return; // do nothing
    }

    if (ws.hasBallTime < GeneralConfig::HasBallValidTime) {
      return; // do nothing
    }

    // compute dynamic kick window depending on distance
    const double theta = std::atan(FieldConfig::GoalSizeX / ws.targetGoalDist);
    const double window = theta;

    const double window_deg = toDeg(window);

    if (!(std::abs(ws.targetGoalRot) < window_deg && ws.targetGoalDist < FieldConfig::kickDistance)) {
      return;
    }

    setKick(true); // queue kick
  }
}
