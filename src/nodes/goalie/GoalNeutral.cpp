#include <nodes/goalie/GoalNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>

namespace GoalNeutral {
  void execute(const WorldState& ws, MotionController* motion) {
    const Vector2 target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    const float rotInput = ws.heading;
    constexpr bool usePID = true;

    auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
  }
}
