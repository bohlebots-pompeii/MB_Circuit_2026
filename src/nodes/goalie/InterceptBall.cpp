#include <nodes/goalie/InterceptBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <cmath>

bool InterceptBall::isDrivingToBall(const WorldState& ws) {
  updateTimers(ws);
  return drivingToBall;
}

void InterceptBall::execute(const WorldState& ws, MotionController* motion) {
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

void InterceptBall::updateTimers(const WorldState& ws) {
  if (ws.ballExists) {
    const Vector2 currentBallVec = ws.ballVec;

    if (const Vector2 diff = currentBallVec - lastBallVec; diff.getMagnitude() > Goalie::BALL_MOVED_THRESH) {
      ballMovementTimer = 0;
      drivingToBall = false;
    }

    lastBallVec = currentBallVec;

    if (!drivingToBall && ballMovementTimer > Goalie::BALL_STATIONARY_MS) {
      drivingToBall = true;
      drivingToBallTimer = 0;
    }
  }
  else {
    ballMovementTimer = 0;
    drivingToBall = false;
  }

  if (drivingToBall && drivingToBallTimer >= Goalie::DRIVE_TO_BALL_MS) {
    drivingToBall = false;
    ballMovementTimer = 0;
  }
}

namespace {
  InterceptBall& getInterceptAction() {
    static InterceptBall action;
    return action;
  }
}

void executeInterceptBall(const WorldState& ws, MotionController* motion) {
  getInterceptAction().execute(ws, motion);
}

bool canExecuteInterceptBall(const WorldState& ws) {
  return getInterceptAction().isDrivingToBall(ws);
}
