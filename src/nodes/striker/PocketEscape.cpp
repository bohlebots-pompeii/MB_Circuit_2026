#include <nodes/striker/PocketEscape.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <cmath>

PocketEscape::PocketEscape(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {}

BT::Status PocketEscape::tick(const WorldState& ws) {
    if (!(ws.hasBall && checkBallInPocket(ws))) {
        return BT::Status::FAILURE;
    }

    const Vector2 target(-15, 0); // (-1, 0) * 15

    MotionController::Output out = _motion->compute(target, 0, false);

    pushData(ws.ena, false, static_cast<int>(out.vx), static_cast<int>(out.vy), 0, 0, false, 0.0);

    return BT::Status::RUNNING;
}

bool PocketEscape::checkBallInPocket(const WorldState& ws) const {
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

