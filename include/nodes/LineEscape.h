#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class LineEscape final : public BT::BehaviorNode {
public:
    explicit LineEscape(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    Vector2 getAwayFromLineVec(const WorldState& ws, int speed) const;
    bool checkBallOnLine(const WorldState& ws) const;
};

