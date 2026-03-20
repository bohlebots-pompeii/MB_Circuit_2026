#include <bt/RoleSelector.h>

namespace BT {

    RoleSelector::RoleSelector(std::string name, std::unique_ptr<BehaviorNode> striker, std::unique_ptr<BehaviorNode> goalie)
        : BehaviorNode(std::move(name)), _striker(std::move(striker)), _goalie(std::move(goalie)) {}

    Status RoleSelector::tick(const WorldState& ws) {
        if (ws.isGoalie) {
            return _goalie->tick(ws);
        }
        return _striker->tick(ws);
    }

}
