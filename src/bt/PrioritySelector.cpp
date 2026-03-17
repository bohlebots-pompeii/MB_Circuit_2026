#include <bt/PrioritySelector.h>

namespace BT {

    void PrioritySelector::addChild(std::unique_ptr<BehaviorNode> child) {
        _children.push_back(std::move(child));
    }

    Status PrioritySelector::tick(const WorldState& ws) {
        for (const auto& child : _children) {
            Status s = child->tick(ws);
            if (s == Status::RUNNING) {
                return Status::RUNNING;
            }
        }
        return Status::FAILURE;
    }

}

