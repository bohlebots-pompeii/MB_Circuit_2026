#include <nodes/striker/GetBehindBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <config/config.h>
#include <cmath>
#include <algorithm>

// helper
inline double wrapAngleRad(double a) {
  while (a > std::numbers::pi) a -= std::numbers::pi * 2;
  while (a < -std::numbers::pi) a += std::numbers::pi * 2;
  return a;
}

static Vector2 getBallPursuitVec(const WorldState& ws) {
  const Vector2 ballVec = ws.ballVec;

  const auto axisBack = Vector2(-1.0, 0.0);
  const auto axisSide = Vector2(0.0, 1.0);

  const double lonDist = Vector2::dotProduct(ballVec * -1.0, axisBack);
  const double latDist = std::abs(Vector2::dotProduct(ballVec * -1.0, axisSide));

  Vector2 ballToRobotVec = ballVec * -1.0;
  ballToRobotVec.normalize();

  const double backComp = Vector2::dotProduct(ballToRobotVec, axisBack);
  const double sideComp = Vector2::dotProduct(ballToRobotVec, axisSide);
  const double sideSign = sideComp >= 0.0 ? 1.0 : -1.0;

  // Updated parameters to match Python defaults
  constexpr double angleWide = 70.0 * std::numbers::pi / 180.0;
  constexpr double angleTight = 90.0 * std::numbers::pi / 180.0;
  constexpr double circleRadiusX = 21.0;
  constexpr double circleRadiusY = 22.0;
  constexpr double frontAngleRad = 45.0 * std::numbers::pi / 180.0;
  constexpr double arcLookahead = 47.0 * std::numbers::pi / 180.0;
  constexpr double slideStart = 218.0;
  constexpr double throughDist = 12.53;

  // Corridor parameters
  constexpr double innerHalfWidthAtBall = ObjectHeights::BALL * 0.0;
  constexpr double outerHalfWidthAtBall = ObjectHeights::BALL * 0.99;
  constexpr double blendHalfWidthAtBall = ObjectHeights::BALL * 0.0;
  constexpr double innerAngle = 0.0 * std::numbers::pi / 180.0;
  constexpr double outerAngle = 17.92 * std::numbers::pi / 180.0;
  constexpr double blendAngle = 0.0 * std::numbers::pi / 180.0;

  const double behindFactor = std::clamp((backComp + 0.5) / 1.5, 0.0, 1.0);
  const double dynamicLimit = angleWide + behindFactor * (angleWide - angleTight);

  double currentAngle = std::atan2(std::abs(sideComp), backComp);
  if (currentAngle > dynamicLimit) currentAngle = dynamicLimit;

  const double nb = std::cos(currentAngle);
  const double ns = std::sin(currentAngle) * sideSign;

  Vector2 circleDirVec = axisBack * nb + axisSide * ns;
  circleDirVec.normalize();

  const double angleFromForward = std::abs(std::atan2(ballToRobotVec.getY(), ballToRobotVec.getX()));
  Vector2 target;
  Vector2 throughPoint = ballVec + Vector2(throughDist, 0.0);

  if (angleFromForward < frontAngleRad) {
    target = ballVec + Vector2(0.0, sideSign * circleRadiusY);
  } else {
    const Vector2 idealCirclePoint = ballVec + Vector2(circleRadiusX * circleDirVec.getX(), circleRadiusY * circleDirVec.getY());
    const double distToCircle = idealCirclePoint.getMagnitude();

    const double idealAngle = std::atan2(circleDirVec.getY(), circleDirVec.getX());
    const double backAngle = std::atan2(axisBack.getY(), axisBack.getX());

    double angleDiff = wrapAngleRad(backAngle - idealAngle);
    angleDiff *= sideSign;
    angleDiff = std::max(0.0, angleDiff + arcLookahead);

    const double lookaheadScale = behindFactor * behindFactor;
    angleDiff *= lookaheadScale;
    angleDiff *= sideSign;

    const double slideT = std::clamp(1.0 - distToCircle / slideStart, 0.0, 1.0);
    const double arcAdvance = slideT * angleDiff;

    const double targetAngle = idealAngle + arcAdvance;
    const Vector2 circlePoint = ballVec + Vector2(circleRadiusX * std::cos(targetAngle), circleRadiusY * std::sin(targetAngle));

    double tAlign = 0.0;
    if (lonDist > -5.0) {
      const double effectiveLon = std::max(0.0, lonDist);
      const double currentInnerWidth = innerHalfWidthAtBall + effectiveLon * std::tan(innerAngle);
      const double currentBlendWidth = blendHalfWidthAtBall + effectiveLon * std::tan(blendAngle);
      const double currentOuterWidth = outerHalfWidthAtBall + effectiveLon * std::tan(outerAngle);

      const bool insideInner = latDist <= currentInnerWidth;
      const bool insideBlend = latDist <= currentBlendWidth;
      if (insideInner || insideBlend) {
        tAlign = 1.0;
      } else {
        tAlign = 1.0 - std::clamp(
          (latDist - currentBlendWidth) / (currentOuterWidth - currentBlendWidth + 1e-5), 0.0, 1.0);
        const double depthBlend = std::clamp((lonDist + 5.0) / 15.0, 0.0, 1.0);
        tAlign *= depthBlend;
      }
    }

    const double tProxRaw = std::clamp(1.0 - (distToCircle - 8.0) / (slideStart - 8.0), 0.0, 1.0);
    const double tProx = tProxRaw * tAlign;
    const double t = std::max(tAlign, tProx);

    target = Vector2::lerp(circlePoint, throughPoint, t);
  }

  double rot = std::abs(ws.ballRot);

  double factor = 2.3;

  if (rot > 60.0) {
    double t = (rot - 60.0) / (180.0 - 60.0);

    // clamp to [0, 1]
    t = std::clamp(t, 0.0, 1.0);

    factor = factor + t * (4.0 - 1.0);
  }

  target *= factor;

  if (80 < rot && rot < 100) {
    target *= 3;
  }

  // NEW: Smooth distance-based boost when bot is behind the ball
  const double ball_distance = ballVec.getMagnitude();
  constexpr double behindBoostDistance = 9.0;  // cm
  constexpr double behindBoostFactor = 2.0;

  if (ballVec.getX() < 0.0) {  // Bot is behind the ball (ball is in front of bot)
    double blend = 0.0;
    if (behindBoostDistance > circleRadiusX) {
      blend = std::clamp((ball_distance - circleRadiusX) / (behindBoostDistance - circleRadiusX), 0.0, 1.0);
    } else {
      blend = (ball_distance < behindBoostDistance) ? 1.0 : 0.0;
    }
    const double multiplier = 1.0 + blend * (behindBoostFactor - 1.0);
    target = target * multiplier;
  }

  if (ballVec.getMagnitude() < 12 && ballVec.getMagnitude() >= 0) {
    if (ballVec.getY() <= 0 && ballVec.getY() > -8) {
      target = Vector2(0.0, 25);
    }
    else if (ballVec.getY() > 0 && ballVec.getY() < 8) {
      target = Vector2(0.0, -20);
    }
  }

  // Ensure target stays on the bot's side of the ball
  if (sideSign > 0.0) {
    target.setY(std::max(target.getY() - 10, ballVec.getY() - 10));
  } else {
    target.setY(std::min(target.getY() + 5, ballVec.getY() + 5));
  }

  return target;
}

static Vector2 getBallApproachVec(const WorldState& ws, const int speed) {
  auto target = ws.ballVec;
  target.normalize();
  return target * speed;
}

static bool checkBallOnLine(const WorldState& ws) {
  const double globalY = ws.globalY;
  const double ballDist = ws.ballDist;

  double globalBallRot = ws.ballRot - ws.heading;
  if (globalBallRot > 180.0) globalBallRot -= 360.0;
  if (globalBallRot < -180.0) globalBallRot += 360.0;

  const double ballRadians = toRad(globalBallRot);
  const double ballGlobalY = globalY + sin(ballRadians) * ballDist;

  if (globalY > FieldConfig::LINE_POS_Y && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -FieldConfig::LINE_POS_Y && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

static bool checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  return std::abs(absoluteGoalDir) > FieldConfig::IN_POCKET_ANGLE;
}

void executeGetBehindBall(const WorldState& ws, MotionController* motion) {
  const bool ballInEdgeCase = checkBallOnLine(ws) || checkBallInPocket(ws);

  Vector2 target;
  double rotInput = 0;
  bool usePID;

  if (ws.peerRunning && ws.globalX < FieldConfig::HARD_BARRIER) {
    target = getToPointVec(ws.globalX, ws.globalY, FieldConfig::HARD_BARRIER + GeneralConfig::BOT_DIAMETER / 2.0,
                           ws.globalY);
    rotInput = ws.heading;
    usePID = false;
  }

  else if (ballInEdgeCase) {
    usePID = false;
    target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);

    if (std::abs(ws.heading) >= GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      if (ws.ballRot > 0.0) {
        rotInput = ws.heading + GeneralConfig::HEADING_HARD_LIMIT_DEG;
      }
      else {
        rotInput = ws.heading - GeneralConfig::HEADING_HARD_LIMIT_DEG;
      }
    }
    else if (std::abs(ws.ballRot) > GeneralConfig::HEADING_HARD_LIMIT_DEG) {
      rotInput = ws.heading;
    }
    else {
      rotInput = ws.ballRot;
    }
  }
  else {
    // normal pursuit
    usePID = true;
    target = getBallPursuitVec(ws);
    rotInput = ws.heading;
  }

  constexpr int dribblerSpeed = 100;

  const auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, true);
}
