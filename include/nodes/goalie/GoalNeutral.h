#pragma once
#include <bt/BehaviorNode.h>
#include <memory>

class MotionController;

class GoalNeutral final : public BT::BehaviorNode {
public:
    explicit GoalNeutral(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;
};

