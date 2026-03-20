#include <nodes/striker/DribbleToGoal.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <config/config.h>

DribbleToGoal::DribbleToGoal(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {
    hasBallTimer = 0;
}

BT::Status DribbleToGoal::tick(const WorldState& ws) {
    if (!ws.hasBall) {
        hasBallTimer = 0;
        return BT::Status::FAILURE;
    }

    if (hasBallTimer <= GeneralConfig::HasBallValidTime) {
        return BT::Status::FAILURE;
    }

    constexpr bool useRotDelta = true;
    float rotInput = 0;

    rotInput = ws.targetGoalRot / 2;
    const Vector2 target = getBallAlignedVec(ws, 50);

    auto [vx, vy, rot] = _motion->compute(target, rotInput, true);

    const int dribblerSpeed = target.getX() > 10 ? 50 : 100;

    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, useRotDelta);

    return BT::Status::RUNNING;
}

Vector2 DribbleToGoal::getBallAlignedVec(const WorldState& ws, int speed) const {
    Vector2 target = degToVec(ws.targetGoalRot);
    target.normalize();
    return target * speed;
}

bool DribbleToGoal::checkBallInPocket(const WorldState& ws) const {
    const double ballX = ws.ballVec.getX();

    float globalGoalDir = ws.targetGoalRot - ws.heading;
    while (globalGoalDir > 180) globalGoalDir -= 360;
    while (globalGoalDir < -180) globalGoalDir += 360;

    if (!ws.ballExists && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
        return true;
    }

    if (ballX > 0 && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
        return true;
    }

    return false;
}

