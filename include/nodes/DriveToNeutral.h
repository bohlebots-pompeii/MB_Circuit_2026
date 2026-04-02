#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class DriveToNeutral final : public BT::BehaviorNode {
public:
    explicit DriveToNeutral(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    static Vector2 getMoveToCenterVec(const WorldState& ws);
};

