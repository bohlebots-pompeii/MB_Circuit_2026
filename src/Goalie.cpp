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
#include <util/MovingAverage.h>
#include <util/helper.h>
#include <config.h>

namespace {
    elapsedMillis ledTimer;
    elapsedMillis positionYAvgTimer;

    MovingAverage<double, 10> positionYAvg;

    // Ball velocity tracking
    MovingAverage<double, 5> ballVelXAvg;
    MovingAverage<double, 5> ballVelYAvg;
    Vector2 lastBallPos{0, 0};
    bool lastBallValid = false;
    unsigned long lastBallTime = 0;
    double lastRobotGlobalX = 0;
    double lastRobotGlobalY = 0;

    // y axis
    double y_Setpoint = 0, y_Input = 0, y_Output = 0;
    PID y_motion(&y_Input, &y_Output, &y_Setpoint, PIDConfig::Y_Kp, 0.05, PIDConfig::Y_Kd, DIRECT);

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

Goalie::Goalie(std::shared_ptr<CM5> cm5, std::shared_ptr<Sensors> sensors, std::shared_ptr<Positioning> positioning)
    : _cm5(std::move(cm5)), _sensors(std::move(sensors)), _positioning(std::move(positioning)),
      _vectorIntersection(std::make_unique<VectorIntersection>()) {
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
    Vector2 middlePointVector = _positioning->getMiddlePointVector();
    const double distance = middlePointVector.getMagnitude();
    middlePointVector.normalize();

    constexpr double MAX_DISTANCE = 30.0f;
    const double ratio = std::min(distance / MAX_DISTANCE, 1.0);
    const double speedFactor = ratio * ratio;
    const int dynamicSpeed = static_cast<int>(speed * speedFactor);

    return middlePointVector * dynamicSpeed;
}

Vector2 Goalie::getInterceptPoint() const {
    constexpr double factor = 0.5; // how far along the predicted path to go (0.5 = halfway)

    const auto ballVec = Vector2(
        cos(toRad(_cm5->getBallRot())) * _cm5->getBallDist(),
        sin(toRad(_cm5->getBallRot())) * _cm5->getBallDist()
    );

    const auto ownGoalVec = Vector2(
        cos(toRad(_cm5->getOwnGoalRot())) * _cm5->getOwnGoalDist(),
        sin(toRad(_cm5->getOwnGoalRot())) * _cm5->getOwnGoalDist()
    );

    const Vector2 goalToBall = ballVec - ownGoalVec;

    return ownGoalVec + goalToBall * factor;
}

void Goalie::printDebugInfo() const {
    // Format: ROBOT_X,ROBOT_Y|BALL_X,BALL_Y|INTERCEPT_X,INTERCEPT_Y|VEL_X,VEL_Y|LINE_SEEN

    // 1. Robot global position
    const double robotX = _cm5->getGlobalX();
    const double robotY = _cm5->getGlobalY();

    // 2. Ball position relative to robot
    double ballX = 0.0;
    double ballY = 0.0;
    if (_cm5->getBallExists()) {
        const double ballAngle = _cm5->getBallRot();
        const double ballDist = _cm5->getBallDist();
        ballX = cos(toRad(ballAngle)) * ballDist;
        ballY = sin(toRad(ballAngle)) * ballDist;
    }

    // 3. Calculated intersection point
    const Vector2 interceptPoint = getInterceptPoint();

    // 5. Line seen status
    const bool lineSeen = _sensors->getLineSeen();

    // Print in format: ROBOT_X,ROBOT_Y|BALL_X,BALL_Y|INTERCEPT_X,INTERCEPT_Y|VEL_X,VEL_Y|LINE_SEEN
    Serial.print("DEBUG:");
    Serial.print(robotX, 2);
    Serial.print(",");
    Serial.print(robotY, 2);
    Serial.print("|");
    Serial.print(ballX, 2);
    Serial.print(",");
    Serial.print(ballY, 2);
    Serial.print("|");
    Serial.print(interceptPoint.getX(), 2);
    Serial.print(",");
    Serial.print(interceptPoint.getY(), 2);
    Serial.print("|");
    Serial.println(lineSeen ? "1" : "0");
}

void Goalie::update() const {
    constexpr int speed = 50.0f;
    static bool CM5_initialized = false;

    _cm5->update();
    updatePositionYAvg(_cm5->getGlobalY());
    _sensors->update();
    _positioning->update();
    setRotDelta(_positioning->getRotationDelta());

    // Print debug info for Python simulator
    printDebugInfo();

    if (!_cm5->getCM5Running()) {
        CM5_initialized = false;
        pushData(false, false, 0, 0, 0, 0);
        _sensors->haltLEDs();
        return;
    }
    if (_cm5->getCM5Running() != CM5_initialized) {
        CM5_initialized = !CM5_initialized;
        _sensors->allLEDsOff();
    }

    // toggle all leds off
    if (!_sensors->getEna()) {
        ledTimer = 0;
    }
    if (ledTimer > 200) {
        _sensors->allLEDsOff();
    }

    Vector2 target;
    double rotInput = 0;
    bool usePID;
    bool kick = false;

    // drive away from line
    if (_sensors->getLineSeen()) {
        if (_sensors->getProgress() < 16) {
            const Vector2 desiredTarget = getInterceptPoint();

            target = driveOnLine(desiredTarget);
        } else {
            target = getAwayFromLineVec(30);
        }

        rotInput = _cm5->getAwayFromOwnGoalAngle();
        usePID = true;
    }

    else {
        // normal ball pursuit - use intercept point prediction
        rotInput = _cm5->getAwayFromOwnGoalAngle();

        // Get predicted intercept point
        const Vector2 interceptPoint = getInterceptPoint();

        target = interceptPoint;
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

    pushData(_sensors->getEna(), kick, static_cast<int>(vx), static_cast<int>(vy), rot, 0);
}