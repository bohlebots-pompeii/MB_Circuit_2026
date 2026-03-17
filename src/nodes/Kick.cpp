#include <nodes/Kick.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <cmath>

Kick::Kick(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {}

BT::Status Kick::tick(const WorldState& ws) {
    if (!(ws.hasBall && std::abs(ws.targetGoalRot) < 15.0f)) {
        return BT::Status::FAILURE;
    }

    constexpr int vx = 0;
    constexpr int vy = 0;
    constexpr int rot = 0;

    pushData(ws.ena, true, vx, vy, rot, 0, true);

    return BT::Status::RUNNING;
}

