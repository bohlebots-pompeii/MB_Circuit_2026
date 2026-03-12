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
#include <config/config.h>
#include <comms/esp-now.h>
#include <util/MovingAverage.h>

namespace {
    elapsedMillis ledTimer;
    elapsedMillis hasBallTimer;
    elapsedMillis ballMovementTimer;
    elapsedMillis drivingToBallTimer;
    elapsedMillis switchWantedCooldownTimer;

    MovingAverage<double, 10> emergencyBallAvgX;
    MovingAverage<double, 10> emergencyBallAvgY;

    MovingAverage<double, 10> strikerAvgX;
    MovingAverage<double, 10> strikerAvgY;

    // y axis
    double y_Setpoint = 0, y_Input = 0, y_Output = 0;
    PID y_motion(&y_Input, &y_Output, &y_Setpoint, 3.0, 0.0, 0.1, DIRECT);

    // x axis
    double x_Setpoint = 0, x_Input = 0, x_Output = 0;
    PID x_motion(&x_Input, &x_Output, &x_Setpoint, 2.5, 0.0, 0.1, DIRECT);

    // rotation
    double rot_Setpoint = 0, rot_Input = 0, rot_Output = 0;
    PID rot_motion(&rot_Input, &rot_Output, &rot_Setpoint, PIDConfig::Rot_Kp, PIDConfig::Rot_Ki, PIDConfig::Rot_Kd, DIRECT);

    bool pidInitialized = false;

    int getRotationControl(const float input) {
        rot_Input = input;
        if (std::abs(rot_Input) < PIDConfig::Rot_deadline) {
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

    auto lineVec = degToVec(lineRot);

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
        const Vector2 overLineComponent = lineNormal * dot * target.getMagnitude();
        result = target - overLineComponent;
    }

    return result;
}

Vector2 Goalie::getHalfCircleTarget(const Vector2* ballVecOverride) const {
    constexpr double HALF_CIRCLE_RADIUS = 60.0;

    const Vector2 ownGoalVec = _cm5->getOwnGoalVec();
    const Vector2 ballVec = ballVecOverride ? *ballVecOverride : _cm5->getBallVec();

    Vector2 goalToBall = ballVec - ownGoalVec;

    if (const double gtbMag = goalToBall.getMagnitude(); gtbMag < 1e-3) {
        goalToBall = ownGoalVec * -1.0;
    }

    goalToBall.normalize();

    Vector2 awayFromGoal = ownGoalVec * -1.0;
    awayFromGoal.normalize();
    const double dot = goalToBall.getX() * awayFromGoal.getX() + goalToBall.getY() * awayFromGoal.getY();
    if (dot < 0) {
        const Vector2 perp(-awayFromGoal.getY(), awayFromGoal.getX());
        const double projPerp = goalToBall.getX() * perp.getX()
                              + goalToBall.getY() * perp.getY();
        goalToBall = perp * (projPerp >= 0 ? 1.0 : -1.0);
    }

    const Vector2 targetOnCircle = ownGoalVec + goalToBall * HALF_CIRCLE_RADIUS;

    return targetOnCircle;
}

// reconstruct ball position
Vector2 Goalie::getEmergencyBallVec() const {
    if constexpr (!USE_COMMUNICATION) { // disabled comms
        return {0, 0};
    }

    const auto& [globalX, globalY, heading, ballRot, ballDist, flags] = espNowGetPeerData();

    // mate sees ball
    if (!espNowPeerAlive() || !espNowGetFlag(flags, 3)) {
        return {0, 0};
    }

    if (ballDist <= 0) {
        return {0, 0};
    }

    const double pGlobalX = globalX;
    const double pGlobalY = globalY;

    const double ballAngleGlobal = toRad(ballRot + heading);
    const double ballGlobalX = pGlobalX + cos(ballAngleGlobal) * ballDist;
    const double ballGlobalY = pGlobalY + sin(ballAngleGlobal) * ballDist;

    const double myGlobalX = _cm5->getGlobalX();
    const double myGlobalY = _cm5->getGlobalY();

    const double diffGlobalX = ballGlobalX - myGlobalX;
    const double diffGlobalY = ballGlobalY - myGlobalY;

    const double myHeadingRad = toRad(_cm5->getHeading()); // rotate back to local for own usage
    const double localX =  diffGlobalX * cos(-myHeadingRad) - diffGlobalY * sin(-myHeadingRad);
    const double localY =  diffGlobalX * sin(-myHeadingRad) + diffGlobalY * cos(-myHeadingRad);

    emergencyBallAvgX.addValue(localX); // try to smooth out
    emergencyBallAvgY.addValue(localY);

    return {emergencyBallAvgX.getAverage(), emergencyBallAvgY.getAverage()};
}

bool Goalie::getSwitchWanted() const {
    if constexpr (!USE_COMMUNICATION) { // if comms disabled
        return false;
    }

    if (!espNowPeerAlive()) {
        return true;
    }

    const auto& [_globalX, _globalY, _heading, _ballRot, _ballDist, _flags] = espNowGetPeerData();

    if (Sensors::getHasBall()) {
        switchWantedCooldownTimer = 0;
        return true;
    }

    if (!espNowGetFlag(_flags, 0)) {
        switchWantedCooldownTimer = 0;
        return true;
    }

    if (!_cm5->getBallExists()) {
        return false;
    }

    if (switchWantedCooldownTimer < 2000) {
        return false;
    }

    /*
    const float pBallDist = _ballDist;
    const float pBallRot = _ballRot;
    const float pHeading = _heading;

    const float oBallDist = _cm5->getBallDist();
    const float oBallRot = _cm5->getBallRot();
    const float oHeading = _cm5->getHeading();

    double gPBallRot = pBallRot - pHeading;
    if (gPBallRot > 180.0) gPBallRot -= 360.0;
    if (gPBallRot < -180.0) gPBallRot += 360.0;

    double gOBallRot = oBallRot - oHeading;
    if (gOBallRot > 180.0) gOBallRot -= 360.0;
    if (gOBallRot < -180.0) gOBallRot += 360.0;

    if (std::abs(gOBallRot) < std::abs(gPBallRot) && oBallDist < pBallDist) {
        switchWantedCooldownTimer = 0;
        return true;
    }
    */

    return false;
}

bool Goalie::checkBallInOwnPocket() const {
    const double ballX = _cm5->getBallVec().getX();

    float globalGoalDir = _cm5->getOwnGoalRot() - _cm5->getHeading();
    while (globalGoalDir > 180) globalGoalDir -= 360;
    while (globalGoalDir < -180) globalGoalDir += 360;

    if (!_cm5->getBallExists() && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
        return true;
    }

    if (ballX > 0 && (globalGoalDir > FieldConfig::FieldPocketAngle || globalGoalDir < -FieldConfig::FieldPocketAngle)) {
        return true;
    }

    return false;
}

void Goalie::update() {
    static Vector2 lastBallVec(0, 0);
    static bool drivingToBall = false;

    int dribbler = 0;

    setRotDelta(_positioning->getRotationDelta());

    if (ledTimer > 200) {
        _sensors->allLEDsOff();
    }

    if (!Sensors::getHasBall()) { hasBallTimer = 0;}

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
            Vector2 desiredTarget;
            if (_cm5->getBallExists()) {
                desiredTarget = getHalfCircleTarget();
            }
            else {
                const double globalX = _cm5->getGlobalX();
                const double globalY = _cm5->getGlobalY();

                desiredTarget = getToPointVec(globalX, globalY, FieldConfig::GoalNeutralPointPositionX, 0);
            }

            target = driveOnLine(desiredTarget);
        } else {
            target = getAwayFromLineVec(30);
        }

        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = true;
    }

    else if (Sensors::getHasBall()) {
        if (hasBallTimer > 200) {
            kick = true;
        }
    }

    else if (!_cm5->getBallExists()) {
        const Vector2 emergencyBall = getEmergencyBallVec();

        if (const double emergencyDist = emergencyBall.getMagnitude(); emergencyDist > 1.0) {
            // rough ball position
            target = getHalfCircleTarget(&emergencyBall);
            rotInput = _cm5->getAwayFromOwnGoalAngle();
            usePID = true;
        } else {
            // drive to neutral point
            const double globalX = _cm5->getGlobalX();
            const double globalY = _cm5->getGlobalY();
            target = getToPointVec(globalX, globalY, FieldConfig::GoalNeutralPointPositionX, 0);
            rotInput = _cm5->getHeading();
            usePID = true;
        }
    }

    else if (drivingToBall) {
        // prevent driving into the ball when going backwards
        if (std::abs(_cm5->getBallRot()) < 80.0) {
            Vector2 toBall = _cm5->getBallVec();
            toBall.normalize();
            target = toBall * 60.0;
        }
        else {
            target = Vector2(0,0);
        }
        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = false;
    }

    else {
        // normal ball pursuit
        target = getHalfCircleTarget();

        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = true;
    }

    // smoothly circle around the ball if too close
    if (_cm5->getBallExists() && !Sensors::getHasBall()) {        const double ballDist = _cm5->getBallDist();
        if (ballDist > 0 && ballDist < BALL_AVOID_DIST && std::abs(_cm5->getBallRot()) > 90.0) {
            Vector2 ballDir = _cm5->getBallVec();
            ballDir.normalize();

            const Vector2 tangentL(-ballDir.getY(),  ballDir.getX());
            const Vector2 tangentR( ballDir.getY(), -ballDir.getX());

            const double dotL = target.getX() * tangentL.getX() + target.getY() * tangentL.getY();
            const double dotR = target.getX() * tangentR.getX() + target.getY() * tangentR.getY();
            const Vector2 tangent = dotL >= dotR ? tangentL : tangentR;

            const double t = 1.0 - ballDist / BALL_AVOID_DIST;
            const double speed = target.getMagnitude();

            Vector2 blended(
                target.getX() / (speed > 1e-3 ? speed : 1.0) * (1.0 - t) + tangent.getX() * t,
                target.getY() / (speed > 1e-3 ? speed : 1.0) * (1.0 - t) + tangent.getY() * t
            );
            blended.normalize();
            target = blended * speed;
        }
    }

    // smoothly avoid the striker using MA(10)-smoothed peer position
    constexpr double STRIKER_AVOID_DIST = 40.0;
    if (espNowPeerAlive()) {
        const auto& peerPkt = espNowGetPeerData();
        const double pGlobalX = peerPkt.globalX;
        const double pGlobalY = peerPkt.globalY;
        const double myGlobalX = _cm5->getGlobalX();
        const double myGlobalY = _cm5->getGlobalY();

        // convert peer global pos to local frame
        const double diffX = pGlobalX - myGlobalX;
        const double diffY = pGlobalY - myGlobalY;
        const double headingRad = toRad(_cm5->getHeading());
        const double localX =  diffX * cos(-headingRad) - diffY * sin(-headingRad);
        const double localY =  diffX * sin(-headingRad) + diffY * cos(-headingRad);

        strikerAvgX.addValue(localX);
        strikerAvgY.addValue(localY);

        const Vector2 strikerLocal(strikerAvgX.getAverage(), strikerAvgY.getAverage());
        const double strikerDist = strikerLocal.getMagnitude();

        if (strikerDist > 1e-3 && strikerDist < STRIKER_AVOID_DIST) {
            Vector2 strikerDir = strikerLocal;
            strikerDir.normalize();

            const Vector2 tangentL(-strikerDir.getY(),  strikerDir.getX());
            const Vector2 tangentR( strikerDir.getY(), -strikerDir.getX());

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

    // speed limit to prevent out of bounds
    _positioning->speedLimit(vx, vy, target);

    if (std::abs(_cm5->getHeading()) > 100) {
        vx *= 0.3;
        vy *= 0.3;
    }

    pushData(_sensors->getEna(), kick, static_cast<int>(vx), static_cast<int>(vy), rot, dribbler, true);
}