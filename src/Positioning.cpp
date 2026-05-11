//
// Created by julius on 06.01.2026.
//

#include "Positioning.h"
#include <memory>
#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include <elapsedMillis.h>
#include "util/MovingAverage.h"
#include <config/config.h>

elapsedMillis rotationDeltaTimer;
elapsedMillis velocityTimer;

MovingAverage<float, 15> velocityXAvg;
MovingAverage<float, 15> velocityYAvg;

std::vector Field = {
  Vector2(-70, -90),
  Vector2(65, -100),
  Vector2(67, 110),
  Vector2(-63, 110)
};

/*
 Stored polygon
  Vector2(-70, -95),
  Vector2(-50, -110),
  Vector2(-30, -94),
  Vector2(40, -94),
  Vector2(55, -120),
  Vector2(88, -95),
  Vector2(80, 110),
  Vector2(60, 120),
  Vector2(40, 100),
  Vector2(-40, 104),
  Vector2(-53, 125),
  Vector2(-75, 125)
 */

Positioning::Positioning(const std::shared_ptr<CM5>& cm5) {
  _cm5 = cm5;
}

void Positioning::update() {
  updateMiddlePointVector();
  updateRotationDelta();
  updateVelocity();
}

void Positioning::updateMiddlePointVector() {
  const float x = _cm5->getGlobalX();
  const float y = _cm5->getGlobalY();

  // Vector from current pos to origin (0,0)
  Vector2 toMiddle(x, y);
  toMiddle.rotate(M_PI);
  _middlePointVector = toMiddle;
}

void Positioning::updateRotationDelta() {
  if (rotationDeltaTimer < 21) {
    return;
  }

  const double rot = _cm5->getHeading();
  _rotationDelta = rot - lastHeading;

  // norm
  if (_rotationDelta > 180.0) _rotationDelta -= 360.0;
  if (_rotationDelta < -180.0) _rotationDelta += 360.0;

  lastHeading = rot;
  rotationDeltaTimer = 0;
}

void Positioning::updateVelocity() {
  if (velocityTimer < 21) {
    return;
  }
  velocityTimer = 0;

  const float x = _cm5->getGlobalX();
  const float y = _cm5->getGlobalY();

  const float dx = x - lastX;
  const float dy = y - lastY;

  velocityXAvg.addValue(dx);
  velocityYAvg.addValue(dy);

  _velocity = Vector2(velocityXAvg.getAverage(), velocityYAvg.getAverage());

  lastX = x;
  lastY = y;
}

void Positioning::speedLimit(float& vx, float& vy, Vector2 _driveVector) const {
  const double x = _cm5->getGlobalX();
  const double y = _cm5->getGlobalY();

  constexpr double lookAheadFrames = 15.0f;

  _driveVector.normalize();

  const double futureX = x + _driveVector.getX() * lookAheadFrames;
  const double futureY = y + _driveVector.getY() * lookAheadFrames;

  const double currentDist = std::hypot(x, y);
  const double futureDist = std::hypot(futureX, futureY);

  if (futureDist <= currentDist) {
    return;
  }

  constexpr float minSpeed = 20.0f;

  double factor = 1.0;
  if (futureDist > SpeedLimiting::maxDistance) {
    factor = 0.0;
  }
  else if (futureDist > SpeedLimiting::slowingDistance) {
    const double normalizedDist = (futureDist - SpeedLimiting::slowingDistance) / (SpeedLimiting::maxDistance -
      SpeedLimiting::slowingDistance);
    factor = 1.0 - normalizedDist * normalizedDist;
  }

  const double newVx = vx * factor;
  const double newVy = vy * factor;

  if (std::abs(vx) >= minSpeed) {
    vx = std::abs(newVx) < minSpeed ? (vx > 0 ? minSpeed : -minSpeed) : newVx;
  }
  if (std::abs(vy) >= minSpeed) {
    vy = std::abs(newVy) < minSpeed ? (vy > 0 ? minSpeed : -minSpeed) : newVy;
  }
}
