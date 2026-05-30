#include <nodes/goalie/HalfCircleGuard.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <cmath>
#include <numbers>

Vector2 getHalfCircleTarget(const WorldState& ws) {
  const Vector2 ownGoalVec = ws.ownGoalVec;
  const Vector2 ballVec = ws.ballVec;

  Vector2 dir = ballVec - ownGoalVec;

  const double dist = dir.getMagnitude();
  dir.normalize();

  double targetDist = std::max(dist * 0.5, Goalie::HALF_CIRCLE_RADIUS);
  targetDist = std::min(targetDist, 112.0); // max is mid point line

  return ownGoalVec + dir * targetDist;
}

Vector2 getAwayFromLineVec(const WorldState& ws) {
  Vector2 lineVec = degToVec(ws.lineRot);
  lineVec.rotate(std::numbers::pi);
  lineVec.normalize();

  Vector2 midVec(-ws.globalX, -ws.globalY);
  if (midVec.getMagnitude() > 1e-3) {
    midVec.normalize();
  }

  Vector2 target = lineVec * 0.3f + midVec * 0.7f;
  target.normalize();
  return target * 30.0;
}

Vector2 driveOnLine(const WorldState& ws, const Vector2& target) {
  Vector2 lineNormal = degToVec(ws.lineRot);
  lineNormal.rotate(std::numbers::pi);
  lineNormal.normalize();

  Vector2 targetDir = target;
  targetDir.normalize();

  const double dot = targetDir.getX() * lineNormal.getX() + targetDir.getY() * lineNormal.getY();
  if (dot < 0) {
    const Vector2 overLineComponent = lineNormal * (dot * target.getMagnitude());
    return target - overLineComponent;
  }
  return target;
}

void applyBallAvoidance(const WorldState& ws, Vector2& target) {
  const double ballDist = ws.ballDist;
  if (!(ballDist > 0 && ballDist < Goalie::BALL_AVOID_DIST && std::abs(ws.ballRot) > 90.0)) {
    return;
  }

  Vector2 ballDir = ws.ballVec;
  ballDir.normalize();

  const Vector2 tangentL(-ballDir.getY(), ballDir.getX());
  const Vector2 tangentR(ballDir.getY(), -ballDir.getX());
  const double dotL = target.getX() * tangentL.getX() + target.getY() * tangentL.getY();
  const double dotR = target.getX() * tangentR.getX() + target.getY() * tangentR.getY();
  const Vector2 tangent = dotL >= dotR ? tangentL : tangentR;

  const double t = 1.0 - ballDist / Goalie::BALL_AVOID_DIST;
  const double speed = target.getMagnitude();
  Vector2 blended(
    target.getX() / (speed > 1e-3 ? speed : 1.0) * (1.0 - t) + tangent.getX() * t,
    target.getY() / (speed > 1e-3 ? speed : 1.0) * (1.0 - t) + tangent.getY() * t
  );
  blended.normalize();
  target = blended * speed;
}

void executeHalfCircleGuard(const WorldState& ws, MotionController* motion) {
  Vector2 target;
  const auto rotInput = ws.heading;
  constexpr bool usePID = true;

  if (ws.lineSeen) {
    if (ws.lineProgress < 16) {
      const Vector2 desiredTarget = getHalfCircleTarget(ws);
      target = driveOnLine(ws, desiredTarget);
    }
    else {
      target = getAwayFromLineVec(ws);
    }
  }
  else {
    target = getHalfCircleTarget(ws);
  }

  applyBallAvoidance(ws, target);

  const int drib = ws.ballDist < 40 && ws.ballDist != 0 ? 100 : 0;
  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID, ws);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
