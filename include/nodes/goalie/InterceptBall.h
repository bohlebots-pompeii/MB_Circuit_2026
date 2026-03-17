#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>
#include <elapsedMillis.h>

class MotionController;

class InterceptBall final : public BT::BehaviorNode {
public:
    explicit InterceptBall(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    elapsedMillis ballMovementTimer;
    elapsedMillis drivingToBallTimer;
    bool drivingToBall;
    Vector2 lastBallVec;

    void updateTimers(const WorldState& ws);
};

