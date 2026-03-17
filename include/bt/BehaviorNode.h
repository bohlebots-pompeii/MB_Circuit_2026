#pragma once

struct WorldState;

namespace BT {

    enum class Status { RUNNING, SUCCESS, FAILURE };

    class BehaviorNode {
    public:
        virtual ~BehaviorNode() = default;
        virtual Status tick(const WorldState& ws) = 0;
    };

}

