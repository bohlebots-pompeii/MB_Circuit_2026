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

    if (!(std::abs(ws.targetGoalRot) < 15.0f)) {
        return BT::Status::FAILURE; // do nothing
    }

    if (ws.hasBallTime < GeneralConfig::HasBallValidTime) {
        return BT::Status::FAILURE; // do nothing
    }

    setKick(true); // queue kick

    return BT::Status::SUCCESS;
}
