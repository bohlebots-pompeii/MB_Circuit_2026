#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class HoldNeutral final : public BT::BehaviorNode {
public:
    explicit HoldNeutral(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;
    Vector2 _lastTarget;
};
