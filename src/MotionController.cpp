#include "MotionController.h"
#include <config/config.h>
#include <cmath>

MotionController* MotionController::_instance = nullptr;

MotionController::MotionController(std::shared_ptr<Positioning> positioning)
    : _xPID(&_xIn, &_xOut, &_xSet, PIDConfig::X_Kp, PIDConfig::X_Ki, PIDConfig::X_Kd, DIRECT),
      _yPID(&_yIn, &_yOut, &_ySet, PIDConfig::Y_Kp, PIDConfig::Y_Ki, PIDConfig::Y_Kd, DIRECT),
      _rotPID(&_rotIn, &_rotOut, &_rotSet, PIDConfig::Rot_Kp, PIDConfig::Rot_Ki, PIDConfig::Rot_Kd, DIRECT),
      _positioning(std::move(positioning))
{
    init();
}

void MotionController::init() {
    if (!_initialized) {
        _ySet = 0.0;
        _xSet = 0.0;
        _rotSet = 0.0;

        _rotPID.SetMode(AUTOMATIC);
        _rotPID.SetOutputLimits(PIDConfig::Rot_OutputMin, PIDConfig::Rot_OutputMax);
        _rotPID.SetSampleTime(PIDConfig::Rot_SampleTime);

        _yPID.SetMode(AUTOMATIC);
        _yPID.SetOutputLimits(PIDConfig::Y_OutputMin, PIDConfig::Y_OutputMax);
        _yPID.SetSampleTime(PIDConfig::Y_SampleTime);

        _xPID.SetMode(AUTOMATIC);
        _xPID.SetOutputLimits(PIDConfig::X_OutputMin, PIDConfig::X_OutputMax);
        _xPID.SetSampleTime(PIDConfig::X_SampleTime);

        _initialized = true;
    }
}

MotionController::Output MotionController::compute(const Vector2& target, const float rotInput, const bool usePID) {
    Output out{};

    _rotIn = rotInput;
    if (std::abs(_rotIn) < PIDConfig::Rot_deadline) {
        _rotIn = 0;
    }
    _rotPID.Compute();
    out.rot = static_cast<int>(_rotOut);

    if (usePID) {
        _xIn = target.getX();
        if (std::isnan(_xIn)) _xIn = 0;
        _xPID.Compute();
        out.vx = -static_cast<float>(_xOut);

        _yIn = target.getY();
        if (std::isnan(_yIn)) _yIn = 0;
        _yPID.Compute();
        out.vy = -static_cast<float>(_yOut);
    } else {
        out.vx = static_cast<float>(target.getX());
        out.vy = static_cast<float>(target.getY());
    }

    _positioning->speedLimit(out.vx, out.vy, target); // @FIXME

    return out;
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
