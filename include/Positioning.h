//
// Created by julius on 06.01.2026.
//

#ifndef BOHLEBOTS_2026_POSITIONING_H
#define BOHLEBOTS_2026_POSITIONING_H

#include <memory>
#include <comms/CM5.h>
#include <Vector2.hpp>

class Positioning {
public:
  explicit Positioning(const std::shared_ptr<CM5> &cm5);

  void update();

  [[nodiscard]] Vector2 getMiddlePointVector() const { return _middlePointVector; }
  void speedLimit(float& vx, float& vy, Vector2 _driveVector) const;
  [[nodiscard]] double getRotationDelta() const { return rotationDelta; }
  [[nodiscard]] Vector2 getVelocity() const { return _velocity; }
private:
  std::shared_ptr<CM5> _cm5;
  Vector2 _middlePointVector;
  Vector2 _velocity;

  double rotationDelta = 0.0f;
  double lastHeading = 0.0f;
  float lastX = 0.0f;
  float lastY = 0.0f;

  void updateMiddlePointVector();
  void updateRotationDelta();
  void updateVelocity();
};


#endif //BOHLEBOTS_2026_POSITIONING_H