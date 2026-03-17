#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class OrbitApproach final : public BT::BehaviorNode {
public:
    explicit OrbitApproach(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    Vector2 getBallPursuitVec(const WorldState& ws) const;
    Vector2 getBallApproachVec(const WorldState& ws, int speed) const;
    Vector2 getBallAlignedVec(const WorldState& ws, int speed) const;
    bool checkBallOnLine(const WorldState& ws) const;
    bool checkBallInPocket(const WorldState& ws) const;
};

