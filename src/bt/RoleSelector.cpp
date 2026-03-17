#include <bt/RoleSelector.h>

namespace BT {

    RoleSelector::RoleSelector(std::unique_ptr<BehaviorNode> striker, std::unique_ptr<BehaviorNode> goalie) : _striker(std::move(striker)), _goalie(std::move(goalie)) {}

    Status RoleSelector::tick(const WorldState& ws) {
        if (ws.isGoalie) {
            return _goalie->tick(ws);
        }
        return _striker->tick(ws);
    }

}

