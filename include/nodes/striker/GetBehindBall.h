#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <util/Vector2.hpp>

class MotionController;

class GetBehindBall final : public BT::BehaviorNode {
public:
    explicit GetBehindBall(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    static Vector2 getBallPursuitVec(const WorldState& ws);
    static Vector2 getBallApproachVec(const WorldState& ws, int speed);
    static Vector2 getBallAlignedVec(const WorldState& ws, int speed);
    static bool checkBallOnLine(const WorldState& ws);
    static bool checkBallInPocket(const WorldState& ws);
};

