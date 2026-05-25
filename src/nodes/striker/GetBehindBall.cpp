#include <nodes/striker/GetBehindBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <util/helper.h>
#include <util/Vector2.hpp>
#include <config/config.h>
#include <cmath>
#include <algorithm>
#include <numbers>

// helper
inline double wrapAngleRad(double a) {
  while (a > std::numbers::pi) a -= std::numbers::pi * 2;
  while (a < -std::numbers::pi) a += std::numbers::pi * 2;
  return a;
}

// ── Tuning constants ─────────────────────────────────────────────────────────

static constexpr double APPROACH_ANGLE_WIDE_RAD = 70.0 * std::numbers::pi / 180.0;
static constexpr double APPROACH_ANGLE_TIGHT_RAD = 90.0 * std::numbers::pi / 180.0;
static constexpr double ELLIPSE_RADIUS_LONGITUDINAL = 21.0; // cm
static constexpr double ELLIPSE_RADIUS_LATERAL = 22.0; // cm
static constexpr double FRONT_CONE_HALF_ANGLE_RAD = 45.0 * std::numbers::pi / 180.0;
static constexpr double ARC_LOOKAHEAD_RAD = 47.0 * std::numbers::pi / 180.0;
static constexpr double ARC_SLIDE_START_DISTANCE = 218.0; // cm — arc-to-through blend begins here
static constexpr double THROUGH_POINT_OFFSET = 12.53; // cm past ball

// Corridor (lateral alignment) parameters
static constexpr double CORRIDOR_INNER_HALF_WIDTH = ObjectHeights::BALL * 0.0;
static constexpr double CORRIDOR_OUTER_HALF_WIDTH = ObjectHeights::BALL * 0.99;
static constexpr double CORRIDOR_BLEND_HALF_WIDTH = ObjectHeights::BALL * 0.0;
static constexpr double CORRIDOR_INNER_FLARE_RAD = 0.0 * std::numbers::pi / 180.0;
static constexpr double CORRIDOR_OUTER_FLARE_RAD = 17.92 * std::numbers::pi / 180.0;
static constexpr double CORRIDOR_BLEND_FLARE_RAD = 0.0 * std::numbers::pi / 180.0;

// Speed-scaling by ball angle relative to robot
static constexpr double BASE_SPEED_FACTOR = 2.3;
static constexpr double BALL_ANGLE_RAMP_START_DEG = 60.0; // below this, base factor applies
static constexpr double BALL_ANGLE_RAMP_MAX_FACTOR = 4.0; // added factor at 180 deg
static constexpr double BALL_ANGLE_BURST_LOW_DEG = 80.0; // burst window: ball is roughly
static constexpr double BALL_ANGLE_BURST_HIGH_DEG = 100.0; //   perpendicular to the robot
static constexpr double BALL_ANGLE_BURST_MULTIPLIER = 3.0;

// Behind-ball boost parameters
static constexpr double BEHIND_BOOST_DISTANCE = 9.0; // cm
static constexpr double BEHIND_BOOST_MAX_MULTIPLIER = 2.0;

// Close-range escape parameters
static constexpr double ESCAPE_TRIGGER_DISTANCE = 12.0; // cm
static constexpr double ESCAPE_LATERAL_DEADZONE = 8.0; // cm — |Y| inside this triggers escape
static constexpr double ESCAPE_TARGET_LATERAL = 25.0; // cm

// ── Helpers ──────────────────────────────────────────────────────────────────

static double computeBehindFactor(const double longitudinalComponent) {
  // 0 when ball is directly ahead of the robot, 1 when squarely behind
  return std::clamp((longitudinalComponent + 0.5) / 1.5, 0.0, 1.0);
}

static double clampApproachAngle(const double rawAngleRad, const double behindFactor) {
  const double dynamicLimit = APPROACH_ANGLE_WIDE_RAD
    + behindFactor * (APPROACH_ANGLE_WIDE_RAD - APPROACH_ANGLE_TIGHT_RAD);
  return std::min(rawAngleRad, dynamicLimit);
}

static double computeAlignmentBlend(const double longitudinalDist, const double lateralDist) {
  // How well the robot is lined up behind the ball laterally.
  // Returns 1 when centred on the approach corridor, 0 when wide of it.
  if (longitudinalDist <= -5.0)
    return 0.0;

  const double effectiveLon = std::max(0.0, longitudinalDist);
  const double innerWidth = CORRIDOR_INNER_HALF_WIDTH + effectiveLon * std::tan(CORRIDOR_INNER_FLARE_RAD);
  const double blendWidth = CORRIDOR_BLEND_HALF_WIDTH + effectiveLon * std::tan(CORRIDOR_BLEND_FLARE_RAD);
  const double outerWidth = CORRIDOR_OUTER_HALF_WIDTH + effectiveLon * std::tan(CORRIDOR_OUTER_FLARE_RAD);

  if (lateralDist <= innerWidth || lateralDist <= blendWidth)
    return 1.0;

  const double lateralFraction = std::clamp(
    (lateralDist - blendWidth) / (outerWidth - blendWidth + 1e-5), 0.0, 1.0);
  const double depthFraction = std::clamp((longitudinalDist + 5.0) / 15.0, 0.0, 1.0);

  return (1.0 - lateralFraction) * depthFraction;
}

static double computeSpeedFactor(const double ballAngleDeg) {
  double factor = BASE_SPEED_FACTOR;

  if (ballAngleDeg > BALL_ANGLE_RAMP_START_DEG) {
    const double t = std::clamp(
      (ballAngleDeg - BALL_ANGLE_RAMP_START_DEG) / (180.0 - BALL_ANGLE_RAMP_START_DEG),
      0.0, 1.0);
    factor += t * (BALL_ANGLE_RAMP_MAX_FACTOR - 1.0);
  }

  return factor;
}

static double computeBehindBoostMultiplier(const Vector2& ballVec) {
  const bool ballIsAhead = ballVec.getX() < 0.0;
  const double ballDistance = ballVec.getMagnitude();

  if (!ballIsAhead)
    return 1.0;

  double blend = 0.0;
  if (BEHIND_BOOST_DISTANCE > ELLIPSE_RADIUS_LONGITUDINAL) {
    blend = std::clamp(
      (ballDistance - ELLIPSE_RADIUS_LONGITUDINAL) / (BEHIND_BOOST_DISTANCE - ELLIPSE_RADIUS_LONGITUDINAL),
      0.0, 1.0);
  }
  else {
    blend = (ballDistance < BEHIND_BOOST_DISTANCE) ? 1.0 : 0.0;
  }

  return 1.0 + blend * (BEHIND_BOOST_MAX_MULTIPLIER - 1.0);
}

static Vector2 computeFrontConeTarget(const Vector2& ballVec, const double sideSign) {
  // Ball is nearly straight ahead: sidestep around it.
  return ballVec + Vector2(0.0, sideSign * ELLIPSE_RADIUS_LATERAL);
}

static Vector2 computeArcTarget(
  const Vector2& ballVec,
  const Vector2& orbitDir,
  const double longitudinalDist,
  const double lateralDist,
  const double behindFactor,
  const double sideSign) {
  const Vector2 idealCirclePoint = ballVec + Vector2(
    ELLIPSE_RADIUS_LONGITUDINAL * orbitDir.getX(),
    ELLIPSE_RADIUS_LATERAL * orbitDir.getY());
  const double distToIdealCirclePoint = idealCirclePoint.getMagnitude();

  // Advance the target along the arc so the robot doesn't stall on approach.
  const double orbitAngle = std::atan2(orbitDir.getY(), orbitDir.getX());
  const double backAxisAngle = std::atan2(-1.0, 0.0); // axisBack = (-1, 0)
  double lookaheadDelta = wrapAngleRad(backAxisAngle - orbitAngle) * sideSign;
  lookaheadDelta = std::max(0.0, lookaheadDelta + ARC_LOOKAHEAD_RAD);
  lookaheadDelta *= behindFactor * behindFactor * sideSign;

  const double slideProgress = std::clamp(
    1.0 - distToIdealCirclePoint / ARC_SLIDE_START_DISTANCE, 0.0, 1.0);
  const double targetAngle = orbitAngle + slideProgress * lookaheadDelta;

  const Vector2 arcPoint = ballVec + Vector2(
    ELLIPSE_RADIUS_LONGITUDINAL * std::cos(targetAngle),
    ELLIPSE_RADIUS_LATERAL * std::sin(targetAngle));

  const Vector2 throughPoint = ballVec + Vector2(THROUGH_POINT_OFFSET, 0.0);

  // Blend from arc point toward through-ball point based on alignment and proximity.
  const double alignmentBlend = computeAlignmentBlend(longitudinalDist, lateralDist);
  const double proximityBlendRaw = std::clamp(
    1.0 - (distToIdealCirclePoint - 8.0) / (ARC_SLIDE_START_DISTANCE - 8.0), 0.0, 1.0);
  const double proximityBlend = proximityBlendRaw * alignmentBlend;
  const double blendToThrough = std::max(alignmentBlend, proximityBlend);

  return Vector2::lerp(arcPoint, throughPoint, blendToThrough);
}

static Vector2 getBallPursuitVec(const WorldState& ws) {
  const Vector2 ballVec = ws.ballVec;

  // Coordinate axes
  const Vector2 axisBack(-1.0, 0.0);
  const Vector2 axisSide(0.0, 1.0);

  Vector2 robotToBallDir = ballVec * -1.0;
  robotToBallDir.normalize();
  const double longitudinalDist = Vector2::dotProduct(ballVec * -1.0, axisBack);
  const double lateralDist = std::abs(Vector2::dotProduct(ballVec * -1.0, axisSide));
  const double longitudinalComponent = Vector2::dotProduct(robotToBallDir, axisBack);
  const double lateralComponent = Vector2::dotProduct(robotToBallDir, axisSide);
  const double sideSign = lateralComponent >= 0.0 ? 1.0 : -1.0;

  // Clamp approach angle based on how far behind the ball the robot is
  const double behindFactor = computeBehindFactor(longitudinalComponent);
  const double rawApproachAngle = std::atan2(std::abs(lateralComponent), longitudinalComponent);
  const double approachAngle = clampApproachAngle(rawApproachAngle, behindFactor);

  // Build the clamped direction toward the ellipse orbit point.
  Vector2 orbitDir = (axisBack * std::cos(approachAngle)
    + axisSide * std::sin(approachAngle) * sideSign);
  orbitDir.normalize();

  const double angleFromForward = std::abs(std::atan2(
    robotToBallDir.getY(), robotToBallDir.getX()));

  Vector2 target;
  if (angleFromForward < FRONT_CONE_HALF_ANGLE_RAD) {
    target = computeFrontConeTarget(ballVec, sideSign);
  }
  else {
    target = computeArcTarget(
      ballVec, orbitDir, longitudinalDist, lateralDist, behindFactor, sideSign);
  }

  // ── Scale by ball angle relative to robot ─────────────────────────────────
  // The further to the side or behind the ball is, the faster the robot pursues.

  const double ballAngleDeg = std::abs(ws.ballRot);
  const double speedFactor = computeSpeedFactor(ballAngleDeg);
  target *= speedFactor;

  if (ballAngleDeg > BALL_ANGLE_BURST_LOW_DEG && ballAngleDeg < BALL_ANGLE_BURST_HIGH_DEG)
    target *= BALL_ANGLE_BURST_MULTIPLIER; // ball is ~perpendicular: maximum urgency

  target *= computeBehindBoostMultiplier(ballVec);

  if (const double distToBall = ballVec.getMagnitude(); distToBall < ESCAPE_TRIGGER_DISTANCE) {
    const double ballY = ballVec.getY();
    if (ballY <= 0.0 && ballY > -ESCAPE_LATERAL_DEADZONE)
      return Vector2(0.0, ESCAPE_TARGET_LATERAL);
    if (ballY > 0.0 && ballY < ESCAPE_LATERAL_DEADZONE)
      return Vector2(0.0, -ESCAPE_TARGET_LATERAL);
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
    usePID = true;
    //target = getBallApproachVec(ws, ws.ballDist < 20.0 ? 15 : 30);
    target = getBallPursuitVec(ws);

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
