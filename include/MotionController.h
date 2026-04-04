#pragma once
#include <util/Vector2.hpp>
#include <PID_v1.h>
#include <memory>
#include "Positioning.h"

class MotionController {
public:
  struct Output {
    float vx;
    float vy;
    int rot;
  };

  explicit MotionController(std::shared_ptr<Positioning> positioning);

  Output compute(const Vector2& target, float rotInput, bool usePID = false);

  [[nodiscard]] Vector2 getLastTarget() const;

  void setRotDeltaRad(double rad);

  [[nodiscard]] double getRotDeltaRad() const;

  static void setInstance(MotionController* instance);
  static MotionController* getInstance();

private:
  double _rotDeltaRad = 0.0;

  double _xIn = 0, _xOut = 0, _xSet = 0;
  double _yIn = 0, _yOut = 0, _ySet = 0;
  double _rotIn = 0, _rotOut = 0, _rotSet = 0;

  PID _xPID;
  PID _yPID;
  PID _rotPID;

  std::shared_ptr<Positioning> _positioning;

  Vector2 _lastTarget{0, 0};

  bool _initialized = false;
  void init();

  static MotionController* _instance;
};
