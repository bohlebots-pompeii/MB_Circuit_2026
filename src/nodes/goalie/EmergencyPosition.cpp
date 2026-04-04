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

void EmergencyPosition::execute(const WorldState& ws, MotionController* motion) {
  const Vector2 emergencyBall = getEmergencyBallVec(ws);
  Vector2 target;
  double rotInput = ws.awayFromOwnGoalAngle;
  bool usePID = true;

  if (emergencyBall.getMagnitude() > 1.0) {
    target = getHalfCircleTarget(ws, emergencyBall);
  }
  else {
    target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::GoalNeutralPointPositionX, 0);
    rotInput = ws.heading;
  }

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, 0, true);
}

Vector2 EmergencyPosition::getEmergencyBallVec(const WorldState& ws) {
  if constexpr (!GeneralConfig::USE_COMMUNICATION) {
    return Vector2(0, 0);
  }

  if (ws.peerBallDist <= 0) {
    return Vector2(0, 0);
  }

  const double ballAngleGlobal = toRad(ws.peerBallRot + ws.peerHeading);
  const double ballGlobalX = ws.peerGlobalX + cos(ballAngleGlobal) * ws.peerBallDist;
  const double ballGlobalY = ws.peerGlobalY + sin(ballAngleGlobal) * ws.peerBallDist;

  const double diffGlobalX = ballGlobalX - ws.globalX;
  const double diffGlobalY = ballGlobalY - ws.globalY;

  const double myHeadingRad = toRad(ws.heading);
  const double localX = diffGlobalX * cos(-myHeadingRad) - diffGlobalY * sin(-myHeadingRad);
  const double localY = diffGlobalX * sin(-myHeadingRad) + diffGlobalY * cos(-myHeadingRad);

  emergencyBallAvgX.addValue(localX);
  emergencyBallAvgY.addValue(localY);

  return Vector2(emergencyBallAvgX.getAverage(), emergencyBallAvgY.getAverage());
}

Vector2 EmergencyPosition::getHalfCircleTarget(const WorldState& ws, const Vector2& ballVec) const {
  const Vector2 ownGoalVec = ws.ownGoalVec;

  Vector2 goalToBall = ballVec - ownGoalVec;
  if (goalToBall.getMagnitude() < 1e-3) {
    goalToBall = ownGoalVec * -1.0;
  }
  goalToBall.normalize();

  Vector2 awayFromGoal = ownGoalVec * -1.0;
  awayFromGoal.normalize();
  const double dot = goalToBall.getX() * awayFromGoal.getX() + goalToBall.getY() * awayFromGoal.getY();
  if (dot < 0) {
    const Vector2 perp(-awayFromGoal.getY(), awayFromGoal.getX());
    const double projPerp = goalToBall.getX() * perp.getX() + goalToBall.getY() * perp.getY();
    goalToBall = perp * (projPerp >= 0 ? 1.0 : -1.0);
  }

  return ownGoalVec + goalToBall * Goalie::HALF_CIRCLE_RADIUS;
}

void executeEmergencyPosition(const WorldState& ws, MotionController* motion) {
  static EmergencyPosition action;
  action.execute(ws, motion);
}
