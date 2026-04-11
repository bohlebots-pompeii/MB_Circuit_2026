#include <nodes/goalie/InterceptBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <util/Vector2.hpp>
#include <elapsedMillis.h>
#include <cmath>

static elapsedMillis g_ballMovementTimer;
static elapsedMillis g_drivingToBallTimer;
static bool g_drivingToBall = false;
static Vector2 g_lastBallVec{0, 0};

static void updateInterceptTimers(const WorldState& ws) {
  if (ws.ballExists) {
    const Vector2 currentBallVec = ws.ballVec;

    if (const Vector2 diff = currentBallVec - g_lastBallVec; diff.getMagnitude() > Goalie::BALL_MOVED_THRESH) {
      g_ballMovementTimer = 0;
      g_drivingToBall = false;
    }

    g_lastBallVec = currentBallVec;

    if (!g_drivingToBall && g_ballMovementTimer > Goalie::BALL_STATIONARY_MS) {
      g_drivingToBall = true;
      g_drivingToBallTimer = 0;
    }
  }
  else {
    g_ballMovementTimer = 0;
    g_drivingToBall = false;
  }

  if (g_drivingToBall && g_drivingToBallTimer >= Goalie::DRIVE_TO_BALL_MS) {
    g_drivingToBall = false;
    g_ballMovementTimer = 0;
  }
}

void executeInterceptBall(const WorldState& ws, MotionController* motion) {
  Vector2 target;
  const auto rotInput = static_cast<float>(ws.awayFromOwnGoalAngle);
  constexpr bool usePID = false;

  // prevent driving into the ball when going backwards
  if (std::abs(ws.ballRot) < 80.0) {
    Vector2 toBall = ws.ballVec;
    toBall.normalize();
    target = toBall * 60.0;
  }
  else {
    target = Vector2(0, 0);
  }

  const int drib = (ws.ballDist < 40 && ws.ballDist != 0) ? 100 : 0;

  auto [vx, vy, rot] = motion->compute(target, rotInput, usePID);
  pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);
}

bool canExecuteInterceptBall(const WorldState& ws) {
  updateInterceptTimers(ws);
  return g_drivingToBall;
}
