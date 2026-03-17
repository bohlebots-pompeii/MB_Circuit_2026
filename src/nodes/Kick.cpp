#include <nodes/Kick.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <cmath>

Kick::Kick(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {}

BT::Status Kick::tick(const WorldState& ws) {
    if (!(ws.hasBall && std::abs(ws.targetGoalRot) < 15.0f)) {
        return BT::Status::FAILURE;
    }

    const Vector2 target = getBallAlignedVec(ws, 100);
    const float rotInput = ws.targetGoalRot / 2.0f;

    auto [vx, vy, rot] = _motion->compute(target, rotInput, false); // usePID = false

    pushData(ws.ena, true, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true, _motion->getRotDeltaRad());

    return BT::Status::RUNNING;
}

Vector2 Kick::getBallAlignedVec(const WorldState& ws, int speed) const {
    Vector2 target = degToVec(ws.targetGoalRot);
    target.normalize();
    return target * speed;
}

