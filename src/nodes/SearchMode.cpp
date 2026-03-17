#include <nodes/SearchMode.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <algorithm>
#include <cmath>

SearchMode::SearchMode(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)) {}

BT::Status SearchMode::tick(const WorldState& ws) {
    // Always running as fallback
    int speed = 50;

    const Vector2 target = getMoveToCenterVec(ws, speed);
    const float rotInput = ws.heading;

    auto [vx, vy, rot] = _motion->compute(target, rotInput, false); // usePID = false

    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true, _motion->getRotDeltaRad());

    return BT::Status::RUNNING;
}

Vector2 SearchMode::getMoveToCenterVec(const WorldState& ws, const int speed) const {
    if (ws.peerAlive) {
        if (!ws.peerRunning) { // !espNowGetFlag(..., 0)
            return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
        }
    }

    if (!ws.peerAlive) {
        return getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    }

    Vector2 middlePointVector(-ws.globalX, -ws.globalY); // Same as Positioning::getMiddlePointVector
    const double distance = middlePointVector.getMagnitude();
    middlePointVector.normalize();

    constexpr double MAX_DISTANCE = 30.0f;
    const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
    const double speedFactor = ratio * ratio;
    const int dynamicSpeed = static_cast<int>(speed * speedFactor);

    return middlePointVector * dynamicSpeed;
}

