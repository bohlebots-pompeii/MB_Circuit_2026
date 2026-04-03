#include <nodes/LineEscape.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <config/config.h>
#include <cmath>
#include <numbers>

LineEscape::LineEscape(std::shared_ptr<MotionController> motion)
    : BT::BehaviorNode("LineEscape"), _motion(std::move(motion)) {}

BT::Status LineEscape::tick(const WorldState& ws) {
    if (!ws.lineSeen) {
        return BT::Status::FAILURE;
    }

    const Vector2 target = getAwayFromLineVec(ws, 30);
    float rotInput = 0;

    double globalBallDir = ws.ballRot - ws.heading;
    while (globalBallDir > 180) globalBallDir -= 360;
    while (globalBallDir < -180) globalBallDir += 360;

    if (checkBallOnLine(ws)) {
        if (std::abs(globalBallDir) < FieldConfig::rotateToBallAngle) {
            rotInput = ws.ballRot;
        } else {
            rotInput = 0.0;
        }
    } else {
        rotInput = 0.0;
    }

    bool kick = false;
    if (std::abs(ws.targetGoalRot) < 15.0 && ws.ena && ws.hasBall) {
        kick = true;
    }

    auto [vx, vy, rot] = _motion->compute(target, rotInput, false);

    pushData(ws.ena, kick, static_cast<int>(vx), static_cast<int>(vy), rot, 100, true);

    return BT::Status::RUNNING;
}

Vector2 LineEscape::getAwayFromLineVec(const WorldState& ws, const int speed) const {
    Vector2 line = degToVec(ws.lineRot);
    line.rotate(std::numbers::pi);

    Vector2 middlePointVector(-ws.globalX, -ws.globalY);
    middlePointVector.normalize();

    line = line * 0.3f + middlePointVector * 0.7f;
    line.normalize();

    return line * speed;
}

bool LineEscape::checkBallOnLine(const WorldState& ws) const {
    const double globalY = ws.globalY;
    const double ballDist = ws.ballDist;

    double globalBallRot = ws.ballRot - ws.heading;
    if (globalBallRot > 180.0) globalBallRot -= 360.0;
    if (globalBallRot < -180.0) globalBallRot += 360.0;

    const double ballRadians = toRad(globalBallRot);
    const double ballGlobalY = globalY + sin(ballRadians) * ballDist;

    if (globalY > FieldConfig::FieldLinePositionY && ballGlobalY > globalY) {
        return true;
    }

    if (globalY < -FieldConfig::FieldLinePositionY && ballGlobalY < globalY) {
        return true;
    }

    return false;
}

