#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>
#include <util/MovingAverage.h>

class MotionController;

class EmergencyPosition final : public BT::BehaviorNode {
public:
    explicit EmergencyPosition(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;
    MovingAverage<double, 10> emergencyBallAvgX;
    MovingAverage<double, 10> emergencyBallAvgY;

    Vector2 getEmergencyBallVec(const WorldState& ws);
    Vector2 getHalfCircleTarget(const WorldState& ws, const Vector2* ballVecOverride) const;
};

