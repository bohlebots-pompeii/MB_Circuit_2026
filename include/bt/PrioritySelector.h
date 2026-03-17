#pragma once
#include <vector>
#include <memory>
#include <bt/BehaviorNode.h>

namespace BT {

    class PrioritySelector final : public BehaviorNode {
    public:
        void addChild(std::unique_ptr<BehaviorNode> child);
        Status tick(const WorldState& ws) override;

    private:
        std::vector<std::unique_ptr<BehaviorNode>> _children;
    };

}

