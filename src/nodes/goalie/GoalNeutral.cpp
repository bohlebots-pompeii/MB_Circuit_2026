#include <nodes/goalie/GoalNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>

GoalNeutral::GoalNeutral(std::shared_ptr<MotionController> motion)
    : BT::BehaviorNode("GoalNeutral"), _motion(std::move(motion)) {}

BT::Status GoalNeutral::tick(const WorldState& ws) {
    const Vector2 target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    const float rotInput = ws.heading;
    constexpr bool usePID = true;

    auto [vx, vy, rot] = _motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);

    return BT::Status::RUNNING;
}

