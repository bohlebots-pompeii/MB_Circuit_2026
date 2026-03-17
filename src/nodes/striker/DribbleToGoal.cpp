#include <nodes/striker/DribbleToGoal.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <config/config.h>
#include <cmath>

DribbleToGoal::DribbleToGoal(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {
    hasBallTimer = 0;
}

BT::Status DribbleToGoal::tick(const WorldState& ws) {
    if (!ws.hasBall) {
        hasBallTimer = 0;
        return BT::Status::FAILURE;
    }

    if (hasBallTimer <= 200) {
        return BT::Status::FAILURE;
    }

    bool usePID = true;
    bool useRotPID = true;
    bool useRotDelta = true;
    Vector2 target;
    int rot = 0;
    float rotInput = 0;
    bool kick = false;

    double globalGoalDir = ws.targetGoalRot - ws.heading;
    while (globalGoalDir > 180) globalGoalDir -= 360;
    while (globalGoalDir < -180) globalGoalDir += 360;

    if (checkBallInPocket(ws) || neutralPointTimer < 2000) {
        usePID = false;
        useRotPID = false;
        useRotDelta = false;
        target = Vector2(-1, 0);
        target *= 15;
        rot = 0;
    }
    else if (std::abs(globalGoalDir) < FieldConfig::FieldPocketAngle && std::abs(globalGoalDir) > FieldConfig::FieldPocketAngle - 10.0) {
        usePID = false;
        useRotPID = false;
        useRotDelta = false;
        target = Vector2(0, 0);
        rot = ws.targetGoalRot > 0 ? -15 : 15;
    }
    else {
        rotInput = ws.targetGoalRot / 2;
        target = getBallAlignedVec(ws, 100);
        usePID = false;
    }

    if (std::abs(ws.targetGoalRot) < 15.0 && ws.ena) {
        kick = true;
    }

    MotionController::Output out;
    if (useRotPID) {
        out = _motion->compute(target, rotInput, usePID);
    } else {
        out = _motion->compute(target, 0, usePID);
        out.rot = rot;
    }

    const double rotDeltaRad = _motion->getRotDeltaRad();

    const int drib = (target.getX() > 10) ? 50 : 100;

    pushData(ws.ena, kick, static_cast<int>(out.vx), static_cast<int>(out.vy), out.rot, drib, useRotDelta, rotDeltaRad);

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

