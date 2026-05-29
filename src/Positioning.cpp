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
#include <WorldState.h>

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

double cross(const Vector2& a, const Vector2& b) { return a.getX() * b.getY() - a.getY() * b.getX(); }

bool isPointInsidePolygon(const Vector2& p, const auto& poly) {
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (poly[i].getY() > p.getY() != poly[j].getY() > p.getY() &&
      p.getX() < (poly[j].getX() - poly[i].getX()) * (p.getY() - poly[i].getY()) / (poly[j].getY() - poly[i].getY()) +
      poly[i].getX()) {
      inside = !inside;
    }
  }
  return inside;
}

Vector2 getNearestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b) {
  const Vector2 ap = p - a;
  const Vector2 ab = b - a;
  double t = (ap.getX() * ab.getX() + ap.getY() * ab.getY()) / (ab.getX() * ab.getX() + ab.getY() * ab.getY());
  t = std::clamp(t, 0.0, 1.0);
  return a + ab * t;
}

Vector2 projectToPolygon(const Vector2& p, const auto& poly) {
  Vector2 closest = p;
  double minDist = 1e9;
  Vector2 bestInwardNormal(0, 0);
  for (size_t i = 0; i < poly.size(); i++) {
    const Vector2 a = poly[i];
    const Vector2 b = poly[(i + 1) % poly.size()];
    const Vector2 proj = getNearestPointOnSegment(p, a, b);
    const double dist = (p - proj).getMagnitude();
    if (dist < minDist) {
      minDist = dist;
      closest = proj;
      const Vector2 s = b - a;
      bestInwardNormal = Vector2(s.getY(), -s.getX());
    }
  }
  bestInwardNormal.normalize();
  return closest + bestInwardNormal * 0.5; // Project slightly inward so we are safely inside
}

double getFirstHitT(const Vector2& pos, const Vector2& future, const Vector2& a, const Vector2& b) {
  const Vector2 r = future - pos;
  const Vector2 s = b - a;

  const double denom = cross(r, s);
  if (std::abs(denom) < 1e-9) return -1.0;

  const double t = cross(a - pos, s) / denom;
  const double u = cross(a - pos, r) / denom;

  // Accept t around 0 to catch exact edge hits, prevent driving outwards if already on edge
  if (t >= -1e-4 && t <= 1.0 && u >= -1e-4 && u <= 1.0001) {
    return std::max(0.0, t);
  }
  return -1.0;
}

double computeSpeedScale(const Vector2& pos, const Vector2& future, int& outHitEdge) {
  const auto& poly = FieldConfig::FIELD_CONTOUR;
  constexpr int n = poly.size();

  double bestT = 1.0;
  outHitEdge = -1;

  for (int i = 0; i < n; ++i) {
    const double t = getFirstHitT(pos, future, poly[i], poly[(i + 1) % n]);
    if  (t >= 0.0 && t < bestT) {
      bestT = t;
      outHitEdge = i;
    }
  }

  return bestT;
}

void Positioning::speedLimit(float& vx, float& vy, const Vector2& _driveVector, const WorldState& ws) const {
  const double rawX = _cm5->getGlobalX();
  const double scaleX = FieldConfig::PERFECT_FIELD_HEIGHT / FieldConfig::REAL_FIELD_HEIGHT;
  const double x = rawX * scaleX;

  const double rawY = _cm5->getGlobalY();
  const double scaleY = FieldConfig::PERFECT_FIELD_WIDTH / FieldConfig::REAL_FIELD_WIDTH;
  const double y = rawY * scaleY;

  Vector2 pos(x, y);
  const Vector2 startingPos = pos;

  Vector2 globalDriveVec = _driveVector.clone();
  //globalDriveVec.rotate(toRad(ws.heading));

  double lookaheadFactor;
  if (!ws.isGoalie) {
    lookaheadFactor = 2.5;
  }
  else {
    lookaheadFactor = 2.0;
  }

  pos = pos + globalDriveVec * lookaheadFactor;

  if (!isPointInsidePolygon(pos, FieldConfig::FIELD_CONTOUR)) {
    pos = projectToPolygon(pos, FieldConfig::FIELD_CONTOUR);
  }

  int hitEdge = -1;
  double factor = computeSpeedScale(startingPos, pos, hitEdge);

  factor = std::clamp(factor, 0.0, 1.0);

  constexpr double minSpeed = 5.0;

  const auto newVx = static_cast<float>(vx * factor);
  const auto newVy = static_cast<float>(vy * factor);

  if (std::abs(vx) >= minSpeed) {
    vx = std::abs(newVx) < minSpeed ? (vx > 0 ? minSpeed : -minSpeed) : newVx;
  }
  if (std::abs(vy) >= minSpeed) {
    vy = std::abs(newVy) < minSpeed ? (vy > 0 ? minSpeed : -minSpeed) : newVy;
  }
}
