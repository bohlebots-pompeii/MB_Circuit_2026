#pragma once
#include <util/Vector2.hpp>
#include <PID_v1.h>

class MotionController {
public:
    struct Output { float vx; float vy; int rot; };

    MotionController();

    // Compute vx/vy/rot from a target vector and rotation input.
    // usePID=true uses the x/y PIDs; usePID=false passes target directly.
    Output compute(const Vector2& target, float rotInput, bool usePID = false);

    // Called once per tick from Bot::update() with the current rotation delta.
    void setRotDeltaRad(double rad);

    // Read by pushData() instead of the removed global.
    [[nodiscard]] double getRotDeltaRad() const;

private:
    double _rotDeltaRad = 0.0;

    double _xIn = 0, _xOut = 0, _xSet = 0;
    double _yIn = 0, _yOut = 0, _ySet = 0;
    double _rotIn = 0, _rotOut = 0, _rotSet = 0;

    PID _xPID;
    PID _yPID;
    PID _rotPID;

    bool _initialized = false;
    void init();
};

