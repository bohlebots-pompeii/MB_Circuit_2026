//
// Created by julius on 06.01.2026.
//

#include "Positioning.h"
#include <memory>
#include <Arduino.h>
#include "Vector2.hpp"
#include <cmath>
#include <algorithm>
#include <elapsedMillis.h>
#include "util/MovingAverage.h"

elapsedMillis rotationDeltaTimer;
elapsedMillis velocityTimer;

MovingAverage<float, 10> velocityXAvg;
MovingAverage<float, 10> velocityYAvg;

std::vector<Vector2> Field = {
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
Positioning::Positioning(const std::shared_ptr<CM5> &cm5) {
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
  if (rotationDeltaTimer < 32) {
    return;
  }
  rotationDeltaTimer = 0;
  const double rot = _cm5->getHeading();
  rotationDelta = rot - lastHeading;
  lastHeading = rot;
}

void Positioning::updateVelocity() {
  if (velocityTimer < 32) {
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

static bool isPointInPolygon(const Vector2& point, const std::vector<Vector2>& polygon) {
  bool inside = false;
  size_t j = polygon.size() - 1;
  for (size_t i = 0; i < polygon.size(); i++) {
    if ((polygon[i].getY() > point.getY()) != (polygon[j].getY() > point.getY()) &&
        (point.getX() < (polygon[j].getX() - polygon[i].getX()) * (point.getY() - polygon[i].getY()) / (polygon[j].getY() - polygon[i].getY()) + polygon[i].getX())) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

static double getDistanceToPolygonEdge(const Vector2& p, const std::vector<Vector2>& polygon) {
  double minDistance = 1e9;

  for (size_t i = 0; i < polygon.size(); i++) {
    Vector2 v = polygon[i];
    Vector2 w = polygon[(i + 1) % polygon.size()];

    const double l2 = pow(w.getX() - v.getX(), 2) + pow(w.getY() - v.getY(), 2);
    if (l2 == 0) continue;

    double t = ((p.getX() - v.getX()) * (w.getX() - v.getX()) + (p.getY() - v.getY()) * (w.getY() - v.getY())) / l2;
    t = std::max(0.0, std::min(1.0, t));

    auto projection = Vector2(v.getX() + t * (w.getX() - v.getX()), v.getY() + t * (w.getY() - v.getY()));

    if (const double dist = std::hypot(p.getX() - projection.getX(), p.getY() - projection.getY()); dist < minDistance) {
      minDistance = dist;
    }
  }
  return minDistance;
}

void Positioning::speedLimit(float& vx, float& vy) const {

  const float x = _cm5->getGlobalX();
  const float y = _cm5->getGlobalY();

  constexpr float lookAheadFrames = 15.0f;
  const float futureX = x + static_cast<float>(_velocity.getX()) * lookAheadFrames;
  const float futureY = y + static_cast<float>(_velocity.getY()) * lookAheadFrames;

  const double currentDist = std::hypot(x, y);
  Serial.println(currentDist);
  const double futureDist = std::hypot(futureX, futureY);
  Serial.println(futureDist);

  if (futureDist <= currentDist) {
    return;
  }

  constexpr double maxDistance = 100.0;
  constexpr double slowingDistance = 60.0;

  double factor = 1.0;

  if (futureDist > maxDistance) {
    vx = constrain(vx, -25.0f, 25.0f);
    vy = constrain(vy, -25.0f, 25.0f);
    return;
  }

  if (futureDist > slowingDistance) {
    const double normalizedDist = (futureDist - slowingDistance) / (maxDistance - slowingDistance);
    factor = 1.0 - (normalizedDist * normalizedDist);
  }

  vx *= static_cast<float>(factor);
  vy *= static_cast<float>(factor);
}



