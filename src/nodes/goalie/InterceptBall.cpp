#include <nodes/goalie/InterceptBall.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <config/config.h>
#include <cmath>

InterceptBall::InterceptBall(std::shared_ptr<MotionController> motion)
    : _motion(std::move(motion)), drivingToBall(false), lastBallVec(0,0) {
    ballMovementTimer = 0;
    drivingToBallTimer = 0;
}

BT::Status InterceptBall::tick(const WorldState& ws) {
    updateTimers(ws);

    if (!(ws.ballExists && drivingToBall)) {
        return BT::Status::FAILURE;
    }

    Vector2 target;
    const float rotInput = ws.awayFromOwnGoalAngle;
    const bool usePID = false;

    // prevent driving into the ball when going backwards
    if (std::abs(ws.ballRot) < 80.0) {
        Vector2 toBall = ws.ballVec;
        toBall.normalize();
        target = toBall * 60.0;
    } else {
        target = Vector2(0,0);
    }

    const int drib = (ws.ballDist < 40 && ws.ballDist != 0) ? 100 : 0;

    auto [vx, vy, rot] = _motion->compute(target, rotInput, usePID);
    pushData(ws.ena, false, static_cast<int>(vx), static_cast<int>(vy), rot, drib, true);

    return BT::Status::RUNNING;
}

void InterceptBall::updateTimers(const WorldState& ws) {
    if (ws.ballExists) {
        const Vector2 currentBallVec = ws.ballVec;
        const Vector2 diff = currentBallVec - lastBallVec;

        if (diff.getMagnitude() > BALL_MOVED_THRESH) {
            ballMovementTimer = 0;
            drivingToBall = false;
        }

        lastBallVec = currentBallVec;

        if (!drivingToBall && ballMovementTimer > BALL_STATIONARY_MS) {
            drivingToBall = true;
            drivingToBallTimer = 0;
        }
    } else {
        ballMovementTimer = 0;
        drivingToBall = false;
    }

    if (drivingToBall && drivingToBallTimer >= DRIVE_TO_BALL_MS) {
        drivingToBall = false;
        ballMovementTimer = 0;
    }
}

