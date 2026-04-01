#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <elapsedMillis.h>
#include <util/Vector2.hpp>

class MotionController;

class DribbleToGoal final : public BT::BehaviorNode {
public:
    explicit DribbleToGoal(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    [[nodiscard]] Vector2 getBallAlignedVec(const WorldState& ws, int speed) const;
};

