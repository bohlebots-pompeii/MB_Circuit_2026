#pragma once
#include <bt/BehaviorNode.h>
#include <memory>

class MotionController;

class PocketEscape final : public BT::BehaviorNode {
public:
    explicit PocketEscape(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    [[nodiscard]] bool checkBallInPocket(const WorldState& ws) const;
};

