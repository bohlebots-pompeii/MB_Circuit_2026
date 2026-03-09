//
// Created by julius on 17.02.2026.
//

#include "Striker.h"
#include <Arduino.h>
#include <../include/util/Vector2.hpp>
#include <motor_mb.h>
#include <numbers>
#include <elapsedMillis.h>
#include <PID_v1.h>
#include <util/MovingAverage.h>
#include <util/helper.h>
#include <config/config.h>

namespace {
    elapsedMillis hasBallTimer;
    elapsedMillis ledTimer;
    elapsedMillis positionYAvgTimer;
    elapsedMillis neutralPointTimer;

    MovingAverage<double, 10> positionYAvg;

    // y axis
    double y_Setpoint = 0, y_Input = 0, y_Output = 0;
    PID y_motion(&y_Input, &y_Output, &y_Setpoint, PIDConfig::Y_Kp, PIDConfig::Y_Ki, PIDConfig::Y_Kd, DIRECT);

    // x axis
    double x_Setpoint = 0, x_Input = 0, x_Output = 0;
    PID x_motion(&x_Input, &x_Output, &x_Setpoint, PIDConfig::X_Kp, PIDConfig::X_Ki, PIDConfig::X_Kd, DIRECT);

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

    void updatePositionYAvg(const double y) {
        if (positionYAvgTimer > 32) {
            positionYAvg.addValue(y);
            positionYAvgTimer = 0;
        }
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

Striker::Striker(std::shared_ptr<CM5> cm5, std::shared_ptr<Sensors> sensors, std::shared_ptr<Positioning> positioning)
    : _cm5(std::move(cm5)), _sensors(std::move(sensors)), _positioning(std::move(positioning)) {
    initPID();
}

void Striker::updateRandomWalk() const {
    static float currentAngle = 0.0f;
    static bool wasOnLine = false;
    static float lastLineRot = 0.0f;
    const bool lineSeen = _sensors->getLineSeen();
    const bool ena = _sensors->getEna();
    constexpr int speed = 30;

    if (lineSeen) {
        const int lineRot = static_cast<int>(_sensors->getLineRot());
        const double lineRotRad = toRad(static_cast<double>(lineRot));
        auto line = Vector2(cos(lineRotRad), sin(lineRotRad));
        line.normalize();
        line *= 30;
        line.rotate(std::numbers::pi);
        const int vx = line.getX();
        const int vy = line.getY();
        wasOnLine = true;
        lastLineRot = _sensors->getLineRot();
        pushData(ena, false, static_cast<int>(round(vx)), static_cast<int>(round(vy)), 0, 0, true);
        return;
    }

    if (wasOnLine) {
        int attempts = 0;
        float newAngle = currentAngle;
        float diff = 0.0f;
        do {
            newAngle = static_cast<float>(random(0, 360));
            diff = std::abs(newAngle - lastLineRot);
            if (diff > 180.0f) diff = 360.0f - diff;
            attempts++;
        } while (diff < 90.0f && attempts < 20);
        currentAngle = newAngle;
        wasOnLine = false;
    }

    const float rad = currentAngle * static_cast<float>(std::numbers::pi / 180.0);
    const float vx = std::cos(rad) * static_cast<float>(speed);
    const float vy = std::sin(rad) * static_cast<float>(speed);

    pushData(ena, false, static_cast<int>(round(vx)), static_cast<int>(round(vy)), 0, 0, true);
}

Vector2 Striker::getAwayFromLineVec(const int speed) const {
    Vector2 line = degToVec(_sensors->getLineRot());
    line.rotate(std::numbers::pi);

    Vector2 middlePointVector = _positioning->getMiddlePointVector();
    middlePointVector.normalize();

    line = line * 0.3f + middlePointVector * 0.7f;
    line.normalize();

    return line * speed;
}

Vector2 Striker::getMoveToCenterVec(const int speed) const {
    Vector2 middlePointVector = _positioning->getMiddlePointVector();
    const double distance = middlePointVector.getMagnitude();
    middlePointVector.normalize();

    constexpr double MAX_DISTANCE = 30.0f;
    const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
    const double speedFactor = ratio * ratio;
    const int dynamicSpeed = static_cast<int>(speed * speedFactor);

    return middlePointVector * dynamicSpeed;
}

Vector2 Striker::getBallAlignedVec(const int speed) const {
    auto target = _cm5->getTargetGoalVec();
    target.normalize();
    return target * speed;
}

Vector2 Striker::getBallApproachVec(const int speed) const {
    auto target = _cm5->getBallVec();
    target.normalize();
    return target * speed;
}

Vector2 Striker::getBallPursuitVec() const {
    const auto ballVec = _cm5->getBallVec();

    const auto targetGoalVec = _cm5->getTargetGoalVec();

    // ball pursuit on straight between ball and goal
    Vector2 ballToGoal = targetGoalVec - ballVec;
    ballToGoal.normalize();

    constexpr double offsetDist = 20.0;
    Vector2 idealPos = ballVec - ballToGoal * offsetDist;

    Vector2 robotToIdeal = idealPos;
    robotToIdeal.normalize();

    Vector2 robotToBall = ballVec;
    robotToBall.normalize();

    const double dot = robotToIdeal.getX() * robotToBall.getX() + robotToIdeal.getY() * robotToBall.getY();

    if (std::abs(_cm5->getBallRot()) > 60.0 && std::abs(dot) > 0.6) {
        const Vector2 perpendicular(-ballToGoal.getY(), ballToGoal.getX());

        const double cross = ballVec.getX() * targetGoalVec.getY() - ballVec.getY() * targetGoalVec.getX();
        const double side = cross > 0 ? 1.0 : -1.0;

        const double shiftStrength = std::clamp((dot - 0.5) * 2.0, 0.0, 1.0);
        constexpr double maxShift = 30.0;

        idealPos = idealPos + perpendicular * (side * maxShift * shiftStrength);
    }

    const Vector2 target = idealPos;
    return target;
}

Vector2 Striker::getToNeutralPointVec() const {
    const double globalX = _cm5->getGlobalX();
    const double globalY = _cm5->getGlobalY();

    Vector2 target;
    bool targetReached = false;

    if (globalY > 0) {
        // check which point to drive to
        target = getToPointVec(globalX, globalY, FieldConfig::NeutralPointPositionX, FieldConfig::NeutralPointPositionY);
        targetReached = getPointReached(globalX, globalY, FieldConfig::NeutralPointPositionX, FieldConfig::NeutralPointPositionY);
    } else {
        target = getToPointVec(globalX, globalY, FieldConfig::NeutralPointPositionX, -FieldConfig::NeutralPointPositionY);
        targetReached = getPointReached(globalX, globalY, FieldConfig::NeutralPointPositionX, -FieldConfig::NeutralPointPositionY);
    }

    if (targetReached) {
        target = Vector2(0,0);
    }

    neutralPointTimer = 0;
    return target;
}

bool Striker::checkBallOnLine() const {
    const double globalY = _cm5->getGlobalY();
    const double ballRot = _cm5->getBallRot();
    const double ballDist = _cm5->getBallDist();

    const double ballRadians = toRad(ballRot);
    const double ballGlobalY = globalY + sin(ballRadians) * ballDist;

    if (globalY > FieldConfig::FieldLinePositionY && ballGlobalY > globalY) {
        return true;
    }

    if (globalY < -FieldConfig::FieldLinePositionY && ballGlobalY < globalY) {
        return true;
    }

    return false;
}

bool Striker::checkBallInPocket() const {
    const double ballX = _cm5->getBallVec().getX();

    float globalGoalDir = _cm5->getTargetGoalRot() - _cm5->getHeading();
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

void Striker::update() const {
    //updateRandomWalk(); // ai vid
    static bool kickOff = false;

    updatePositionYAvg(_cm5->getGlobalY());
    setRotDelta(_positioning->getRotationDelta());

    /*
    if (!kickOff && _sensors->getEna()) {
        pushData(_sensors->getEna(), false, 50, 0, 0, 0);
        delay(300);
        kickOff = true;
        pushData(_sensors->getEna(), true, 50, 0, 0, 0);
        delay(100);
    }
    */

    if (ledTimer > 200) {
        _sensors->allLEDsOff();
    }

    // flags
    bool usePID = true;
    bool useRotPID = true;
    bool kick = false;
    bool useRotDelta = true;
    int speed = 50;

    // inits
    int rot = 0;
    Vector2 target;
    double rotInput = 0;

    // because used multiple times
    double globalBallDir = _cm5->getBallRot() - _cm5->getHeading();
    while (globalBallDir > 180) globalBallDir -= 360;
    while (globalBallDir < -180) globalBallDir += 360;

    if (!Sensors::getHasBall()) {
        hasBallTimer = 0;
    }

    // drive away from line
    if (_sensors->getLineSeen()) {
        target = getAwayFromLineVec(30);
        if (checkBallOnLine()) {
            if (std::abs(globalBallDir) < FieldConfig::rotateToBallAngle) {
                rotInput = _cm5->getBallRot();
            }
            else {
                rotInput = _cm5->getHeading();
            }
        }
        else {
            rotInput = _cm5->getHeading();
        }

        if (std::abs(_cm5->getTargetGoalRot()) < 15.0 && _sensors->getEna() && Sensors::getHasBall() && _cm5->getTargetGoalDist() < FieldConfig::kickDistance) {
            kick = true;
        }
        usePID = false;
    }

    // drive to goal if the bot has the ball
    else if (Sensors::getHasBall()) {
        if (hasBallTimer > 200) {

            // get ball out of pocket
            if (checkBallInPocket() || neutralPointTimer < 2000) {
                target = Vector2(-10, 0);
                target.normalize();
                target *= 20;
                rotInput = 0;
                usePID = false;
            }

            // align with goal and shoot
            else {
                const double target_angle = _cm5->getTargetGoalRot();
                if (std::abs(target_angle) > 10) {
                    useRotPID = false;
                    if (target_angle < 0) {
                        target = Vector2(0,0);
                        rot = 20;
                    } else {
                        target = Vector2(0,0);
                        rot = -20;
                    }
                }
                if (std::abs(target_angle) < 45 && _cm5->getTargetGoalDist() > FieldConfig::kickDistance) {
                    target = degToVec(_cm5->getTargetGoalRot());
                    target *= 40;
                }
            }

            if (std::abs(_cm5->getTargetGoalRot()) < 15.0 && _sensors->getEna() && _cm5->getTargetGoalDist() < FieldConfig::kickDistance) {
                kick = true;
            }
        }
    }

    // drive to midPoint if ball not seen
    else if (!_cm5->getBallExists()) {
        target = getMoveToCenterVec(speed);
        rotInput = _cm5->getHeading();
        usePID = false;
    }

    // else pursue ball
    else {
        // approach ball
        if (std::abs(_cm5->getBallRot()) < 10 && _cm5->getBallDist() < 30.0) {
            target = getBallApproachVec(30);
            if (std::abs(globalBallDir) < FieldConfig::rotateToBallAngle) {
                rotInput = _cm5->getBallRot();
            }
            else {
                rotInput = _cm5->getHeading();
            }
            usePID = false;
        }

        // pursue ball
        else {
            // rotate to ball
            if (checkBallOnLine() || checkBallInPocket()) {
                if (std::abs(globalBallDir) < FieldConfig::rotateToBallAngle) {
                    rotInput = _cm5->getBallRot();
                }
                else {
                    rotInput = _cm5->getHeading();
                }
                if (_cm5->getBallDist() < 20.0) { speed = 15; }
                else { speed = 30; }
                target = getBallApproachVec(speed);
                usePID = false;
            }

            // normal ball pursuit
            else {
                rotInput = _cm5->getTargetGoalRot();
                target = getBallPursuitVec();
            }
        }
    }

    // update pids
    if (useRotPID) {
        rot = getRotationControl(static_cast<float>(rotInput));
    }

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

    if (std::abs(_cm5->getHeading()) > 100) {
        vx *= 0.3;
        vy *= 0.3;
    }

    int drib;
    if (target.getX() > 0) {
        drib = 50;
    } else {
        drib = 100;
    }

    pushData(_sensors->getEna(), kick, static_cast<int>(vx), static_cast<int>(vy), rot, drib, useRotDelta);
}