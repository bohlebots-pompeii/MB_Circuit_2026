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

MovingAverage<double, 15> velocityXAvg;
MovingAverage<double, 15> velocityYAvg;

Positioning::Positioning(const std::shared_ptr<CM5>& cm5) {
  _cm5 = cm5;
}

void Positioning::update() {
  updateMiddlePointVector();
  updateRotationDelta();
  updateVelocity();
}

void Positioning::updateMiddlePointVector() {
  const double x = _cm5->getGlobalX();
  const double y = _cm5->getGlobalY();

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

  const double x = _cm5->getGlobalX();
  const double y = _cm5->getGlobalY();

  const double dx = x - lastX;
  const double dy = y - lastY;

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
  if (futureDist > SpeedLimiting::MAX_DIST) {
    factor = 0.0;
  }
  else if (futureDist > SpeedLimiting::SLOWING_DIST) {
    const double normalizedDist = (futureDist - SpeedLimiting::SLOWING_DIST) / (SpeedLimiting::MAX_DIST -
      SpeedLimiting::SLOWING_DIST);
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
