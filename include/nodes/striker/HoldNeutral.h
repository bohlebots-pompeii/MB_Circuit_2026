#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>
#include <elapsedMillis.h>

class MotionController;

class HoldNeutral final : public BT::BehaviorNode {
public:
    explicit HoldNeutral(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;
    elapsedMillis middlePointTimer;
    Vector2 _lastTarget;

    Vector2 getMoveToCenterVec(const WorldState& ws, int speed) const;
};

