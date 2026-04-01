#pragma once
#include <bt/BehaviorNode.h>

class Kick final : public BT::BehaviorNode {
public:
    explicit Kick();
    BT::Status tick(const WorldState& ws) override;
};