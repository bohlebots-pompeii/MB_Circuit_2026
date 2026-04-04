#include <nodes/goalie/HalfCircleGuard.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <util/MovingAverage.h>
#include <cmath>
#include <numbers>

static MovingAverage<double, 10> g_strikerAvgX;
static MovingAverage<double, 10> g_strikerAvgY;

static Vector2 getHalfCircleTarget(const WorldState& ws) {
  const Vector2 ownGoalVec = ws.ownGoalVec;
  const Vector2 ballVec = ws.ballVec;

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

static Vector2 getAwayFromLineVec(const WorldState& ws) {
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

static Vector2 driveOnLine(const WorldState& ws, const Vector2& target) {
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

static void applyBallAvoidance(const WorldState& ws, Vector2& target) {
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

static void applyStrikerAvoidance(const WorldState& ws, Vector2& target) {
  if (!ws.peerAlive) {
    return;
  }

  const double diffX = ws.peerGlobalX - ws.globalX;
  const double diffY = ws.peerGlobalY - ws.globalY;
  const double headingRad = toRad(ws.heading);
  const double localX = diffX * cos(-headingRad) - diffY * sin(-headingRad);
  const double localY = diffX * sin(-headingRad) + diffY * cos(-headingRad);

  g_strikerAvgX.addValue(localX);
  g_strikerAvgY.addValue(localY);

  const Vector2 strikerLocal(g_strikerAvgX.getAverage(), g_strikerAvgY.getAverage());
  const double strikerDist = strikerLocal.getMagnitude();
  constexpr double STRIKER_AVOID_DIST = 40.0;
  if (!(strikerDist > 1e-3 && strikerDist < STRIKER_AVOID_DIST)) {
    return;
  }

  Vector2 strikerDir = strikerLocal;
  strikerDir.normalize();

  const Vector2 tangentL(-strikerDir.getY(), strikerDir.getX());
  const Vector2 tangentR(strikerDir.getY(), -strikerDir.getX());
  const double dotL = target.getX() * tangentL.getX() + target.getY() * tangentL.getY();
  const double dotR = target.getX() * tangentR.getX() + target.getY() * tangentR.getY();
  const Vector2 tangent = dotL >= dotR ? tangentL : tangentR;

  const double t = 1.0 - strikerDist / STRIKER_AVOID_DIST;
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
  const float rotInput = static_cast<float>(ws.awayFromOwnGoalAngle);
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
  applyStrikerAvoidance(ws, target);

  const int drib = (ws.ballDist < 40 && ws.ballDist != 0) ? 100 : 0;
  auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}
