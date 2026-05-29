#include "MotionController.h"
#include <config/config.h>
#include <cmath>
#include <WorldState.h>

MotionController* MotionController::_instance = nullptr;

MotionController::MotionController(std::shared_ptr<Positioning> positioning, bool goalie)
  : _xPID(&_xIn, &_xOut, &_xSet, PIDConfig::X_Kp, PIDConfig::X_Ki, PIDConfig::X_Kd, DIRECT),
    _yPID(&_yIn, &_yOut, &_ySet, PIDConfig::Y_Kp, PIDConfig::Y_Ki, PIDConfig::Y_Kd, DIRECT),
    _rotPID(&_rotIn, &_rotOut, &_rotSet, PIDConfig::ROTATION_Kp, PIDConfig::ROTATION_Ki, PIDConfig::ROTATION_Kd,
            DIRECT),
    _positioning(std::move(positioning)) {
  if (!goalie) { init(); }
  else { initGoalie(); }
}

void MotionController::init() {
  if (!_initialized) {
    _ySet = 0.0;
    _xSet = 0.0;
    _rotSet = 0.0;

    // built PID's from config
    _rotPID.SetMode(AUTOMATIC);
    _rotPID.SetOutputLimits(PIDConfig::ROTATION_O_MIN, PIDConfig::ROTATION_O_MAX);
    _rotPID.SetSampleTime(PIDConfig::ROTATION_SAMPLE_T);

    _yPID.SetMode(AUTOMATIC);
    _yPID.SetOutputLimits(PIDConfig::Y_O_MIN, PIDConfig::Y_O_MAX);
    _yPID.SetSampleTime(PIDConfig::Y_SampleTime);

    _xPID.SetMode(AUTOMATIC);
    _xPID.SetOutputLimits(PIDConfig::X_O_MIN, PIDConfig::X_O_MAX);
    _xPID.SetSampleTime(PIDConfig::X_SAMPLE_T);

    _xPID.SetDTermFilter(true, 0.4);
    _yPID.SetDTermFilter(true, 0.4);

    _initialized = true;
  }
}

void MotionController::initGoalie() {
  if (!_initialized) {
    _ySet = 0.0;
    _xSet = 0.0;
    _rotSet = 0.0;

    // built PID's from config
    _rotPID.SetMode(AUTOMATIC);
    _rotPID.SetOutputLimits(GoaliePIDConfig::ROTATION_O_MIN, GoaliePIDConfig::ROTATION_O_MAX);
    _rotPID.SetSampleTime(GoaliePIDConfig::ROTATION_SAMPLE_T);

    _yPID.SetMode(AUTOMATIC);
    _yPID.SetOutputLimits(GoaliePIDConfig::Y_O_MIN, GoaliePIDConfig::Y_O_MAX);
    _yPID.SetSampleTime(GoaliePIDConfig::Y_SAMPLE_T);

    _xPID.SetMode(AUTOMATIC);
    _xPID.SetOutputLimits(GoaliePIDConfig::X_O_MIN, GoaliePIDConfig::X_O_MAX);
    _xPID.SetSampleTime(GoaliePIDConfig::X_SAMPLE_T);

    _initialized = true;
  }
}

MotionController::Output MotionController::compute(const Vector2& target, const float rotInput, const bool usePID,
                                                   const WorldState& ws) {
  Output out{};

  _lastTarget = target;

  _rotIn = rotInput;
  if (std::abs(_rotIn) < PIDConfig::ROTATION_DEADZONE) {
    // low pass filter
    _rotIn = 0;
  }
  _rotPID.Compute();
  out.rot = static_cast<int>(_rotOut);

  if (usePID) {
    _xIn = target.getX();
    if (std::isnan(_xIn)) _xIn = 0; // init check
    _xPID.Compute();
    out.vx = -static_cast<float>(_xOut);

    _yIn = target.getY();
    if (std::isnan(_yIn)) _yIn = 0;
    _yPID.Compute();
    out.vy = -static_cast<float>(_yOut);
  }
  else {
    out.vx = static_cast<float>(target.getX()); // linear motion
    out.vy = static_cast<float>(target.getY());
  }

  if (!ws.ena) {
    _rotPID.ResetIntegral();
    _xPID.ResetIntegral();
    _yPID.ResetIntegral();
  }

  _positioning->speedLimit(out.vx, out.vy, target, ws);

  return out;
}

Vector2 MotionController::getLastTarget() const {
  return _lastTarget;
}

void MotionController::setRotDeltaRad(const double rad) {
  _rotDeltaRad = rad;
}

double MotionController::getRotDeltaRad() const {
  return _rotDeltaRad;
}

void MotionController::setInstance(MotionController* instance) {
  _instance = instance;
}

MotionController* MotionController::getInstance() {
  return _instance;
}
