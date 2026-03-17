#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class Kick final : public BT::BehaviorNode {
public:
    explicit Kick(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    // Helper
    [[nodiscard]] Vector2 getBallAlignedVec(const WorldState& ws, int speed) const;
};
