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

  constexpr double angleWide = 90 * std::numbers::pi / 180.0;
  constexpr double angleTight = 45.0 * std::numbers::pi / 180.0;
  const double behindFactor = std::clamp((backComp + 0.5) / 1.5, 0.0, 1.0);
  const double dynamicLimit = angleTight + behindFactor * (angleWide - angleTight);

  double currentAngle = std::atan2(std::abs(sideComp), backComp);
  if (currentAngle > dynamicLimit) currentAngle = dynamicLimit;

  const double nb = std::cos(currentAngle);
  const double ns = std::sin(currentAngle) * sideSign;

  Vector2 circleDirVec = axisBack * nb + axisSide * ns;
  circleDirVec.normalize();

  constexpr double circleRadius = 22.0;
  const Vector2 idealCirclePoint = ballVec + circleDirVec * circleRadius;
  const double distToCircle = idealCirclePoint.getMagnitude();

  const double idealAngle = std::atan2(circleDirVec.getY(), circleDirVec.getX());
  const double backAngle = std::atan2(axisBack.getY(), axisBack.getX());

  constexpr double arcLookahead = 40.0 * std::numbers::pi / 180.0;
  double angleDiff = wrapAngleRad(backAngle - idealAngle);
  angleDiff *= sideSign;
  angleDiff = std::max(0.0, angleDiff + arcLookahead);

  const double lookaheadScale = behindFactor * behindFactor; // quadratic ease-in
  angleDiff *= lookaheadScale;

  angleDiff *= sideSign;

  constexpr double slideStart = 20.0;
  const double slideT = std::clamp(1.0 - distToCircle / slideStart, 0.0, 1.0);
  const double arcAdvance = slideT * angleDiff;

  const double targetAngle = idealAngle + arcAdvance;
  const Vector2 circlePoint = ballVec + Vector2(std::cos(targetAngle), std::sin(targetAngle)) * circleRadius;

  constexpr double corridorInner = ObjectHeights::BALL * 2.0;
  constexpr double corridorOuter = corridorInner * 2.0;

  double tAlign = 0.0;
  if (lonDist > -5.0) {
    const double lateralAlignment = 1.0 - std::clamp(
      (latDist - corridorInner) / (corridorOuter - corridorInner), 0.0, 1.0);
    const double depthBlend = std::clamp((lonDist + 5.0) / 15.0, 0.0, 1.0);
    tAlign = lateralAlignment * depthBlend;
  }

  const double tProxRaw = std::clamp(1.0 - (distToCircle - 8.0) / (slideStart - 8.0), 0.0, 1.0);
  const double tProx = tProxRaw * tAlign;
  const double t = std::max(tAlign, tProx);

  constexpr double throughDist = 20.0;
  const Vector2 throughPoint = ballVec + Vector2(throughDist, 0.0);

  const Vector2 target = Vector2::lerp(circlePoint, throughPoint, t);
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
