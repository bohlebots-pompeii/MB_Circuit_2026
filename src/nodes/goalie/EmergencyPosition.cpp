//
// created by Julius on 04.04.26
//

#include <nodes/goalie/EmergencyPosition.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <cmath>
#include <util/MovingAverage.h>

namespace EmergencyPosition {
  Vector2 getEmergencyBallVec(const WorldState& ws);
  Vector2 getHalfCircleTarget(const WorldState& ws, const Vector2* ballVecOverride);

  static MovingAverage<double, 10> emergencyBallAvgX;
  static MovingAverage<double, 10> emergencyBallAvgY;

  void execute(const WorldState& ws, MotionController* motion) {
    if (!(!ws.ballExists && ws.peerBallValid && ws.peerAlive)) {
      return;
    }

    const Vector2 emergencyBall = getEmergencyBallVec(ws);
    Vector2 target;
    double rotInput = ws.awayFromOwnGoalAngle;
    bool usePID = true;

    if (const double emergencyDist = emergencyBall.getMagnitude(); emergencyDist > 1.0) {
      target = getHalfCircleTarget(ws, &emergencyBall);
    }
    else {
      target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
      rotInput = ws.heading;
      usePID = true;
    }

    auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
  }

  Vector2 getEmergencyBallVec(const WorldState& ws) {
    if constexpr (!GeneralConfig::USE_COMMUNICATION) {
      return Vector2(0, 0);
    }

    if (ws.peerBallDist <= 0) {
      return Vector2(0, 0);
    }

    const double pGlobalX = ws.peerGlobalX;
    const double pGlobalY = ws.peerGlobalY;

    const double ballAngleGlobal = toRad(ws.peerBallRot + ws.peerHeading);
    const double ballGlobalX = pGlobalX + cos(ballAngleGlobal) * ws.peerBallDist;
    const double ballGlobalY = pGlobalY + sin(ballAngleGlobal) * ws.peerBallDist;

    const double myGlobalX = ws.globalX;
    const double myGlobalY = ws.globalY;

    const double diffGlobalX = ballGlobalX - myGlobalX;
    const double diffGlobalY = ballGlobalY - myGlobalY;

    const double myHeadingRad = toRad(ws.heading);
    const double localX = diffGlobalX * cos(-myHeadingRad) - diffGlobalY * sin(-myHeadingRad);
    const double localY = diffGlobalX * sin(-myHeadingRad) + diffGlobalY * cos(-myHeadingRad);

    emergencyBallAvgX.addValue(localX);
    emergencyBallAvgY.addValue(localY);

    return Vector2(emergencyBallAvgX.getAverage(), emergencyBallAvgY.getAverage());
  }

  Vector2 getHalfCircleTarget(const WorldState& ws, const Vector2* ballVecOverride) {
    const Vector2 ownGoalVec = ws.ownGoalVec;
    const Vector2 ballVec = *ballVecOverride;

    Vector2 goalToBall = ballVec - ownGoalVec;

    if (const double gtbMag = goalToBall.getMagnitude(); gtbMag < 1e-3) {
      goalToBall = ownGoalVec * -1.0;
    }
    goalToBall.normalize();

    Vector2 awayFromGoal = ownGoalVec * -1.0;
    awayFromGoal.normalize();
    if (const double dot = goalToBall.getX() * awayFromGoal.getX() + goalToBall.getY() * awayFromGoal.getY(); dot < 0) {
      const Vector2 perp(-awayFromGoal.getY(), awayFromGoal.getX());
      const double projPerp = goalToBall.getX() * perp.getX() + goalToBall.getY() * perp.getY();
      goalToBall = perp * (projPerp >= 0 ? 1.0 : -1.0);
    }

    return ownGoalVec + goalToBall * Goalie::HALF_CIRCLE_RADIUS;
  }
}
