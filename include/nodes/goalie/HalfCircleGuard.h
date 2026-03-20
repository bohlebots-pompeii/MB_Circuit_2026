#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>
#include <util/MovingAverage.h>

class MotionController;

class HalfCircleGuard final : public BT::BehaviorNode {
public:
    explicit HalfCircleGuard(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;
    MovingAverage<double, 10> strikerAvgX;
    MovingAverage<double, 10> strikerAvgY;

    static Vector2 getHalfCircleTarget(const WorldState& ws);
    [[nodiscard]] static Vector2 getAwayFromLineVec(const WorldState& ws);
    [[nodiscard]] static Vector2 driveOnLine(const WorldState& ws, const Vector2& target);
    static void applyBallAvoidance(const WorldState& ws, Vector2& target);
    void applyStrikerAvoidance(const WorldState& ws, Vector2& target);
};

