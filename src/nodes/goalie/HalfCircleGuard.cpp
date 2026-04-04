#include <nodes/goalie/HalfCircleGuard.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/helper.h>
#include <cmath>
#include <numbers>
#include <util/MovingAverage.h>

namespace HalfCircleGuard {
  Vector2 getHalfCircleTarget(const WorldState& ws);
  Vector2 getAwayFromLineVec(const WorldState& ws);
  Vector2 driveOnLine(const WorldState& ws, const Vector2& target);
  void applyBallAvoidance(const WorldState& ws, Vector2& target);
  void applyStrikerAvoidance(const WorldState& ws, Vector2& target);

  static MovingAverage<double, 10> strikerAvgX;
  static MovingAverage<double, 10> strikerAvgY;


  void execute(const WorldState& ws, MotionController* motion) {
    if (!(ws.ballExists && !ws.hasBall)) {
      return;
    }

    Vector2 target;
    const float rotInput = ws.awayFromOwnGoalAngle;
    constexpr bool usePID = true;

    // Line logic
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
      // Normal guarding
      target = getHalfCircleTarget(ws);
    }

    applyBallAvoidance(ws, target);
    applyStrikerAvoidance(ws, target);

    const int drib = ws.ballDist < 40 && ws.ballDist != 0 ? 100 : 0;

    auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
  }

  Vector2 getHalfCircleTarget(const WorldState& ws) {
    constexpr double HALF_CIRCLE_RADIUS = 60.0;

    const Vector2 ownGoalVec = ws.ownGoalVec;
    const Vector2 ballVec = ws.ballVec;

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

    return ownGoalVec + goalToBall * HALF_CIRCLE_RADIUS;
  }

  Vector2 getAwayFromLineVec(const WorldState& ws) {
    const double lineRot = ws.lineRot;
    auto lineVec = degToVec(lineRot);
    lineVec.rotate(std::numbers::pi);
    lineVec.normalize();

    Vector2 midVec(-ws.globalX, -ws.globalY);
    midVec.normalize();

    Vector2 target = lineVec * 0.3f + midVec * 0.7f;
    target.normalize();
    target *= 30; // default speed
    return target;
  }

  Vector2 driveOnLine(const WorldState& ws, const Vector2& target) {
    const double lineRot = ws.lineRot;
    Vector2 lineNormal = degToVec(lineRot);
    lineNormal.rotate(std::numbers::pi);
    lineNormal.normalize();

    Vector2 targetDir = target;
    targetDir.normalize();

    const double dot = targetDir.getX() * lineNormal.getX() + targetDir.getY() * lineNormal.getY();

    Vector2 result = target;
    if (dot < 0) {
      const Vector2 overLineComponent = lineNormal * dot * target.getMagnitude();
      result = target - overLineComponent;
    }
    return result;
  }

  void applyBallAvoidance(const WorldState& ws, Vector2& target) {
    const double ballDist = ws.ballDist;
    if (ballDist > 0 && ballDist < Goalie::BALL_AVOID_DIST && std::abs(ws.ballRot) > 90.0) {
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
  }

  void applyStrikerAvoidance(const WorldState& ws, Vector2& target) {
    if (ws.peerAlive) {
      const double pGlobalX = ws.peerGlobalX;
      const double pGlobalY = ws.peerGlobalY;
      const double myGlobalX = ws.globalX;
      const double myGlobalY = ws.globalY;

      const double diffX = pGlobalX - myGlobalX;
      const double diffY = pGlobalY - myGlobalY;
      const double headingRad = toRad(ws.heading);
      const double localX = diffX * cos(-headingRad) - diffY * sin(-headingRad);
      const double localY = diffX * sin(-headingRad) + diffY * cos(-headingRad);

      strikerAvgX.addValue(localX);
      strikerAvgY.addValue(localY);

      const Vector2 strikerLocal(strikerAvgX.getAverage(), strikerAvgY.getAverage());
      const double strikerDist = strikerLocal.getMagnitude();

      if (constexpr double STRIKER_AVOID_DIST = 40.0; strikerDist > 1e-3 && strikerDist < STRIKER_AVOID_DIST) {
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
    }
  }
}
