#include <bt/PrioritySelector.h>
#include <Arduino.h>

#include "config/config.h"

namespace BT {

    void PrioritySelector::addChild(std::unique_ptr<BehaviorNode> child) {
        _children.push_back(std::move(child));
    }

    Status PrioritySelector::tick(const WorldState& ws) {
        for (const auto& child : _children) {
            if (const Status s = child->tick(ws); s == Status::RUNNING) {
                Serial.println(child->getName().c_str());
                return Status::RUNNING;
            }
        }
        return Status::FAILURE;
    }

}

