//
// Created by julius on 06.01.2026.
//

#pragma once

#include <memory>
#include <comms/CM5.h>
#include <util/Vector2.hpp>
#include <WorldState.h>

struct WorldState;

class Positioning {
public:
  explicit Positioning(const std::shared_ptr<CM5>& cm5);

  void update();

  [[nodiscard]] Vector2 getMiddlePointVector() const { return _middlePointVector; }
  void speedLimit(float& vx, float& vy, const Vector2& _driveVector, const WorldState& ws) const;
  [[nodiscard]] double getRotationDelta() const { return _rotationDelta; }
  [[nodiscard]] Vector2 getVelocity() const { return _velocity; }

private:
  std::shared_ptr<CM5> _cm5;
  Vector2 _middlePointVector;
  Vector2 _velocity;

  double _rotationDelta = 0.0f;
  double lastHeading = 0.0f;
  double lastX = 0.0f;
  double lastY = 0.0f;

  void updateMiddlePointVector();
  void updateRotationDelta();
  void updateVelocity();
};
