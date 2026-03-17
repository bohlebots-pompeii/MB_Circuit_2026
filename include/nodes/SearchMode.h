#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class SearchMode final : public BT::BehaviorNode {
public:
    explicit SearchMode(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    Vector2 getMoveToCenterVec(const WorldState& ws, int speed) const;
};

