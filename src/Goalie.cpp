//
// Created by julius on 17.02.2026.
//

#include "Goalie.h"
#include <Arduino.h>
#include <../include/util/Vector2.hpp>
#include <motor_mb.h>
#include <numbers>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <util/helper.h>
#include <config.h>

namespace {
    elapsedMillis ledTimer;
    elapsedMillis hasBallTimer;
    elapsedMillis ballMovementTimer;
    elapsedMillis drivingToBallTimer;

    // y axis
    double y_Setpoint = 0, y_Input = 0, y_Output = 0;
    PID y_motion(&y_Input, &y_Output, &y_Setpoint, 3.0, PIDConfig::Y_Ki, PIDConfig::Y_Kd, DIRECT);

    // x axis
    double x_Setpoint = 0, x_Input = 0, x_Output = 0;
    PID x_motion(&x_Input, &x_Output, &x_Setpoint, 3.0, PIDConfig::X_Ki, PIDConfig::X_Kd, DIRECT);

    // rotation
    double rot_Setpoint = 0, rot_Input = 0, rot_Output = 0;
    PID rot_motion(&rot_Input, &rot_Output, &rot_Setpoint, PIDConfig::Rot_Kp, PIDConfig::Rot_Ki, PIDConfig::Rot_Kd, DIRECT);

    bool pidInitialized = false;

    int getRotationControl(const float input) {
        rot_Input = input;
        if (std::abs(rot_Input) < PIDConfig::Rot_Deadzone) {
            rot_Input = 0;
        }
        rot_motion.Compute();
        return static_cast<int>(rot_Output);
    }

    void updateYMotion(const double y) {
        y_Input = y;
        if (isnan(y_Input)) y_Input = 0;
        y_motion.Compute();
    }

    void updateXMotion(const double x) {
        x_Input = x;
        if (isnan(x_Input)) x_Input = 0;
        x_motion.Compute();
    }

    void initPID() {
        if (!pidInitialized) {
            y_Setpoint = 0.0;
            x_Setpoint = 0.0;
            rot_Setpoint = 0.0;

            rot_motion.SetMode(AUTOMATIC);
            rot_motion.SetOutputLimits(PIDConfig::Rot_OutputMin, PIDConfig::Rot_OutputMax);
            rot_motion.SetSampleTime(PIDConfig::Rot_SampleTime);

            y_motion.SetMode(AUTOMATIC);
            y_motion.SetOutputLimits(PIDConfig::Y_OutputMin, PIDConfig::Y_OutputMax);
            y_motion.SetSampleTime(PIDConfig::Y_SampleTime);

            x_motion.SetMode(AUTOMATIC);
            x_motion.SetOutputLimits(PIDConfig::X_OutputMin, PIDConfig::X_OutputMax);
            x_motion.SetSampleTime(PIDConfig::X_SampleTime);

            pidInitialized = true;
        }
    }
}

Goalie::Goalie(std::shared_ptr<CM5> cm5, std::shared_ptr<Sensors> sensors, std::shared_ptr<Positioning> positioning)
    : _cm5(std::move(cm5)), _sensors(std::move(sensors)), _positioning(std::move(positioning)) {
    initPID();
}

Vector2 Goalie::getAwayFromLineVec(const int speed) const {
    const double lineRot = _sensors->getLineRot();
    const double lineDist = _sensors->getProgress();

    auto lineVec = Vector2(cos(lineRot / std::numbers::pi * 180.0f) * lineDist, sin(lineRot / std::numbers::pi * 180.0f) * lineDist);

    lineVec.rotate(std::numbers::pi);
    lineVec.normalize();
    auto midVec = _positioning->getMiddlePointVector();
    midVec.normalize();

    Vector2 target = lineVec * 0.3f + midVec * 0.7f;
    target.normalize();
    target *= speed;

    return target;
}

Vector2 Goalie::driveOnLine(const Vector2& target) const {
    // Away from line component
    const double lineRot = _sensors->getLineRot();
    Vector2 lineNormal = degToVec(lineRot);
    lineNormal.rotate(std::numbers::pi);
    lineNormal.normalize();

    Vector2 targetDir = target;
    targetDir.normalize();

    // Project target onto line normal to get component going over the line
    // dot > 0 means moving away from line
    // dot < 0 means moving over the line
    const double dot = targetDir.getX() * lineNormal.getX() + targetDir.getY() * lineNormal.getY();

    Vector2 result = target;

    if (dot < 0) {
        // Keep only the perpendicular component
        Vector2 overLineComponent = lineNormal * dot * target.getMagnitude();
        result = target - overLineComponent;
    }

    return result;
}

Vector2 Goalie::getMoveToCenterVec(const int speed) const {
    constexpr double HALF_CIRCLE_RADIUS = 60.0;

    const Vector2 ownGoalVec = _cm5->getOwnGoalVec();

    Vector2 awayFromGoal = ownGoalVec * -1.0;
    awayFromGoal.normalize();

    // halfCircleCenter is relative to robot (like all CM5 vectors)
    Vector2 target = ownGoalVec + awayFromGoal * HALF_CIRCLE_RADIUS;

    target.normalize();
    target *= speed;

    return target;
}

Vector2 Goalie::getHalfCircleTarget() const {
    constexpr double HALF_CIRCLE_RADIUS = 60.0;

    const Vector2 ownGoalVec = _cm5->getOwnGoalVec();

    const Vector2 ballVec = _cm5->getBallVec();

    Vector2 goalToBall = ballVec - ownGoalVec;

    if (const double gtbMag = goalToBall.getMagnitude(); gtbMag < 1e-3) {
        goalToBall = ownGoalVec * -1.0;
    }

    goalToBall.normalize();

    Vector2 awayFromGoal = ownGoalVec * -1.0;
    awayFromGoal.normalize();
    const double dot = goalToBall.getX() * awayFromGoal.getX()
                     + goalToBall.getY() * awayFromGoal.getY();
    if (dot < 0) {
        const Vector2 perp(-awayFromGoal.getY(), awayFromGoal.getX());
        const double projPerp = goalToBall.getX() * perp.getX()
                              + goalToBall.getY() * perp.getY();
        goalToBall = perp * (projPerp >= 0 ? 1.0 : -1.0);
    }

    const Vector2 targetOnCircle = ownGoalVec + goalToBall * HALF_CIRCLE_RADIUS;

    return targetOnCircle;
}

void Goalie::update() const {
    static bool CM5_initialized = false;
    static Vector2 lastBallVec(0, 0);
    static bool drivingToBall = false;

    constexpr unsigned long BALL_STATIONARY_MS = 2000;
    constexpr unsigned long DRIVE_TO_BALL_MS = 600;
    constexpr double BALL_MOVED_THRESH = 5.0;

    int dribbler = 0;

    setRotDelta(_positioning->getRotationDelta());

    if (!_cm5->getCM5Running() != CM5_initialized) {
        CM5_initialized = _cm5->getCM5Running();
        _sensors->allLEDsOff();
    }

    if (ledTimer > 200) {
        _sensors->allLEDsOff();
    }

    if (!_sensors->getHasBall()) { hasBallTimer = 0;}

    if (_cm5->getBallDist() < 40 && _cm5->getBallDist() != 0) { dribbler = 100; }

    if (_cm5->getBallExists()) {
        const Vector2 currentBallVec = _cm5->getBallVec();
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

    Vector2 target;
    double rotInput = 0;
    bool usePID = false;
    bool kick = false;

    // drive away from line
    if (_sensors->getLineSeen()) {
        if (_sensors->getProgress() < 16) {
            const Vector2 desiredTarget = getHalfCircleTarget();

            target = driveOnLine(desiredTarget);
        } else {
            target = getAwayFromLineVec(30);
        }

        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = true;
    }

    else if (_sensors->getHasBall()) {
        if (hasBallTimer > 100) {
            kick = true;
            target = Vector2(50, 0);
        }
    }

    else if (!_cm5->getBallExists()) {
        target = getMoveToCenterVec(40);
        usePID = true;
    }

    else if (drivingToBall) {
        Vector2 toBall = _cm5->getBallVec();
        toBall.normalize();
        target = toBall * 60.0;
        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = false;
    }

    else {
        // normal ball pursuit - use intercept point prediction
        target = getHalfCircleTarget();

        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = true;
    }

    // update pids
    const int rot = getRotationControl(static_cast<float>(rotInput));

    updateXMotion(target.getX());
    updateYMotion(target.getY());

    float vx = 0;
    float vy = 0;

    if (usePID) {
        vx = static_cast<float>(-x_Output);
        vy = static_cast<float>(-y_Output);
    } else {
        vx = static_cast<float>(target.getX());
        vy = static_cast<float>(target.getY());
    }

    _positioning->speedLimit(vx, vy, target);

    pushData(_sensors->getEna(), kick, static_cast<int>(vx), static_cast<int>(vy), rot, dribbler);
}