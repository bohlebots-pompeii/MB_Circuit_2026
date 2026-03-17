#pragma once
#include <memory>
#include <bt/BehaviorNode.h>
#include <WorldState.h>

namespace BT {

    class RoleSelector final : public BehaviorNode {
    public:
        RoleSelector(std::unique_ptr<BehaviorNode> striker,
                     std::unique_ptr<BehaviorNode> goalie);

        Status tick(const WorldState& ws) override;

    private:
        std::unique_ptr<BehaviorNode> _striker;
        std::unique_ptr<BehaviorNode> _goalie;
    };

}

