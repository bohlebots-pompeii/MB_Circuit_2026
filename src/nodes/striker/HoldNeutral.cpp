#include <nodes/striker/HoldNeutral.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <cmath>
#include <algorithm>

namespace HoldNeutral {
  static Vector2 _lastTarget(0, 0);


  void execute(const WorldState& ws, MotionController* motion) {
    if (ws.lastBallSeenTime > 500) {
      return;
    }
    Serial.println(ws.lastBallSeenTime);

    Vector2 target = _lastTarget;
    constexpr int rotInput = 0;
    bool usePID = false;

    _lastTarget = target;

    /* // disabled for testing
    if (ws.peerRunning && ws.globalX < -70) {
       Vector2 mv(-ws.globalX, -ws.globalY);
       mv.normalize();
       target = mv * 20.0f;
       usePID = false;
    }
    */

    constexpr int drib = 100;

    auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
  }
}
