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
  Vector2 ballToGoalVec = ws.targetGoalVec - ws.ballVec;
  ballToGoalVec.normalize();

  const auto axisBack = ballToGoalVec * -1.0;
  const auto axisSide = Vector2(-ballToGoalVec.getY(), ballToGoalVec.getX());

  Vector2 ballToRobotVec = ballVec * -1.0;
  ballToRobotVec.normalize();

  const double backComp = Vector2::dotProduct(ballToRobotVec, axisBack);
  const double sideComp = Vector2::dotProduct(ballToRobotVec, axisSide);

  const double sideSign = sideComp >= 0.0 ? 1.0 : -1.0;

  // tight and far limit
  constexpr double angleWide = 90 * std::numbers::pi / 180.0;
  constexpr double angleTight = 45.0 * std::numbers::pi / 180.0;

  // determine how far behind the ball we are
  const double behindFactor = std::clamp((backComp + 0.5) / 1.5, 0.0, 1.0);

  // linear interpolation
  const double dynamicLimit = angleTight + behindFactor * (angleWide - angleTight);

  // hemisphere angle clamping
  double currentAngle = std::atan2(std::abs(sideComp), backComp);
  if (currentAngle > dynamicLimit) {
    currentAngle = dynamicLimit;
  }

  const double nb = std::cos(currentAngle);
  const double ns = std::sin(currentAngle) * sideSign;

  // ideal circle direction
  Vector2 circleDirVec = axisBack * nb + axisSide * ns;
  circleDirVec.normalize();

  constexpr double circleRadius = 25.0;
  const Vector2 idealCirclePoint = ballVec + circleDirVec * circleRadius;
  const double distToCircle = idealCirclePoint.getMagnitude();

  const double idealAngle = std::atan2(circleDirVec.getY(), circleDirVec.getX());
  const double backAngle = std::atan2(axisBack.getY(), axisBack.getX());

  double angleDiff = wrapAngleRad(backAngle - idealAngle);

  angleDiff *= sideSign;
  angleDiff = std::max(0.0, angleDiff);
  angleDiff *= sideSign;

  constexpr double slideStart = 20.0;
  const double slideT = std::clamp(1.0 - distToCircle / slideStart, 0.0, 1.0);
  const double arcAdvance = slideT * angleDiff;

  const double targetAngle = idealAngle + arcAdvance;
  const Vector2 circlePoint = ballVec + Vector2(std::cos(targetAngle), std::sin(targetAngle)) * circleRadius;

  double effectiveSideComp = std::abs(sideComp);
  if (ws.ballDist < circleRadius) {
    effectiveSideComp *= ws.ballDist / circleRadius;
  }
  const double alignment = backComp * (1.0 - effectiveSideComp);

  // blending
  constexpr double alignStart = 0.70;
  constexpr double alignFull = 0.90;
  constexpr double throughDist = 20.0;

  const double tAlign = std::clamp((alignment - alignStart) / (alignFull - alignStart), 0.0, 1.0);

  const double tProxRaw = std::clamp(1.0 - (distToCircle - 8.0) / (slideStart - 8.0), 0.0, 1.0);
  const double tProx = tProxRaw * tAlign;

  const double t = std::max(tAlign, tProx);

  const Vector2 throughPoint = ballVec + ballToGoalVec * throughDist;

  // linear interpolation
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

  if (globalY > FieldConfig::FieldLinePositionY && ballGlobalY > globalY) {
    return true;
  }

  if (globalY < -FieldConfig::FieldLinePositionY && ballGlobalY < globalY) {
    return true;
  }

  return false;
}

static bool checkBallInPocket(const WorldState& ws) {
  double absoluteGoalDir = ws.targetGoalRot - ws.heading;
  while (absoluteGoalDir > 180.0) absoluteGoalDir -= 360.0;
  while (absoluteGoalDir < -180.0) absoluteGoalDir += 360.0;

  return std::abs(absoluteGoalDir) > FieldConfig::FieldPocketAngle;
}

void executeGetBehindBall(const WorldState& ws, MotionController* motion) {
  /*
  double globalBallDir = ws.ballRot - ws.heading;
  while (globalBallDir > 180) globalBallDir -= 360;
  while (globalBallDir < -180) globalBallDir += 360;

  const bool ballAligned = std::abs(globalBallDir) < FieldConfig::rotateToBallAngle;
  const bool ballInEdgeCase = checkBallOnLine(ws) || checkBallInPocket(ws);

  Vector2 target;
  double rotInput = 0;
  bool usePID;

  if (ballAligned) {
    target = getBallApproachVec(ws, ws.ballDist > 30.0 ? 50 : 30);
    rotInput = ws.ballRot;
    usePID = false;
  }
  else if (ballInEdgeCase) {
    target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);
    rotInput = ws.ballRot;
    usePID = false;
  }
  else {
    // normal pursuit
    target = getBallPursuitVec(ws);
    rotInput = ws.heading;
    usePID = true;
  }

  constexpr int dribblerSpeed = 100;
  */

  const double rotInput = ws.heading;
  const Vector2 target = getBallPursuitVec(ws);

  Serial.print(target.getX());
  Serial.print(" ");
  Serial.println(target.getY());

  const bool usePID = true;
  const int dribblerSpeed = 100;

  auto [vx, vy, rot] = motion->compute(target, static_cast<float>(rotInput), usePID);

  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, dribblerSpeed, true);
}
