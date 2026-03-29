#include <nodes/striker/HoldNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <cmath>
#include <algorithm>

HoldNeutral::HoldNeutral(std::shared_ptr<MotionController> motion)
    : BT::BehaviorNode("HoldNeutral"), _motion(std::move(motion)), _lastTarget(0,0) {
    middlePointTimer = 0;
}

BT::Status HoldNeutral::tick(const WorldState& ws) {
    if (ws.ballExists) {
        middlePointTimer = 0;
    }

    Vector2 target;
    float rotInput = 0;
    bool usePID = false;
    int speed = 50;

    if (middlePointTimer < 500) {
        target = _lastTarget;
        rotInput = 0;
        usePID = false;
    } else {
        target = getMoveToCenterVec(ws, speed);
        rotInput = ws.heading;
        usePID = false;
    }

    _lastTarget = target;

    if (ws.peerRunning && ws.globalX < -70) {
       Vector2 mv(-ws.globalX, -ws.globalY);
       mv.normalize();
       target = mv * 20.0f;
       usePID = false;
    }

    constexpr int drib = 70;

    auto [vx, vy, rot] = _motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);

    return BT::Status::RUNNING;
}

Vector2 HoldNeutral::getMoveToCenterVec(const WorldState& ws, const int speed) {
    if (ws.peerAlive && !ws.peerRunning) {
        return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    }

    if (!ws.peerAlive) {
        return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    }

    Vector2 middlePointVector(-ws.globalX, -ws.globalY);
    const double distance = middlePointVector.getMagnitude();
    middlePointVector.normalize();

    constexpr double MAX_DISTANCE = 30.0f;
    const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
    const double speedFactor = ratio * ratio;
    const int dynamicSpeed = static_cast<int>(speed * speedFactor);

    return middlePointVector * dynamicSpeed;
}


