#pragma once
#include <string>

struct WorldState;

namespace BT {

    enum class Status { RUNNING, SUCCESS, FAILURE };

    class BehaviorNode {
    public:
        explicit BehaviorNode(std::string name) : _name(std::move(name)) {}
        virtual ~BehaviorNode() = default;
        virtual Status tick(const WorldState& ws) = 0;

        [[nodiscard]] const std::string& getName() const { return _name; }

    private:
        std::string _name;
    };

}
