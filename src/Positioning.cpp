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

elapsedMillis rotationDeltaTimer;

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
  if (rotationDeltaTimer < 30) {
    return;
  }
  rotationDeltaTimer = 0;
  const double rot = _cm5->getHeading();
  rotationDelta = rot - lastHeading;
  lastHeading = rot;
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

int Positioning::speedLimit(const Vector2& driveVector, const int driveSpeed) const {
  if (const int heading = _cm5->getHeading(); abs(heading) > 50) {
    return 0;
  }

  const int x = _cm5->getGlobalX();
  const int y = _cm5->getGlobalY();
  Vector2 currentPos(x, y);

  const double headingRad = _cm5->getHeading() * M_PI / 180.0;

  Vector2 globalDriveVector = driveVector;
  globalDriveVector.normalize();
  globalDriveVector.rotate(headingRad);

  const double dotProduct = globalDriveVector.getX() * _middlePointVector.getX() +
                            globalDriveVector.getY() * _middlePointVector.getY();

  if (dotProduct >= -30.0) {
    return driveSpeed;
  }

  Vector2 lookAheadVector = driveVector;
  lookAheadVector.normalize();
  lookAheadVector *= 30.0;
  lookAheadVector.rotate(headingRad);

  const Vector2 futurePos = currentPos + lookAheadVector;

  /*
  if (!isPointInPolygon(futurePos, Field)) {
    Serial.println("Point is not in Polygon");
    return 0;
  }
  Serial.println("Point is in Polygon");
  */

  const double distToEdge = getDistanceToPolygonEdge(futurePos, Field);

  if (constexpr double slowingDistance = 20.0; distToEdge < slowingDistance) {
    const double factor = distToEdge / slowingDistance;
    return static_cast<int>(driveSpeed * factor);
  }

  return driveSpeed;
}



