#include <nodes/Kick.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <cmath>
#include <config/config.h>

Kick::Kick()
    : BehaviorNode("Kick") {}

// decider if the bot should kick or not
BT::Status Kick::tick(const WorldState& ws) {
    setKick(false);

    if (!ws.hasBall) {
        return BT::Status::FAILURE; // do nothing
    }

    if (ws.hasBallTime < GeneralConfig::HasBallValidTime) {
        return BT::Status::FAILURE; // do nothing
    }

    // compute dynamic kick window depending on distance
    const double theta = std::atan(FieldConfig::GoalSizeX / ws.targetGoalDist);
    const double window = theta;

    const double window_deg = toDeg(window);

    if (!(std::abs(ws.targetGoalRot) < window_deg && ws.targetGoalDist < FieldConfig::kickDistance)) {
        return BT::Status::FAILURE;
    }

    setKick(true); // queue kick

    return BT::Status::SUCCESS;
}
