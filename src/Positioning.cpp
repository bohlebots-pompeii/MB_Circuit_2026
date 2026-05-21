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

double cross(const Vector2& a, const Vector2& b) { return a.getX() * b.getY() - a.getY() * b.getX(); }

int orient(const Vector2& a, const Vector2& b, const Vector2& c) {
  const double v = cross(b - a, c - a);
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

bool onSegment(const Vector2& a, const Vector2& b, const Vector2& p) {
  return std::min(a.getX(), b.getX()) <= p.getX() && p.getX() <= std::max(a.getX(), b.getX()) &&
    std::min(a.getY(), b.getY()) <= p.getY() && p.getY() <= std::max(a.getY(), b.getY());
}

bool segmentsIntersect(const Vector2& a, const Vector2& b,
                       const Vector2& c, const Vector2& d) {
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);

  if (o1 != o2 && o3 != o4) return true;

  // collinear cases
  if (o1 == 0 && onSegment(a, b, c)) return true;
  if (o2 == 0 && onSegment(a, b, d)) return true;
  if (o3 == 0 && onSegment(c, d, a)) return true;
  if (o4 == 0 && onSegment(c, d, b)) return true;

  return false;
}

double getFirstHitT(const Vector2& pos, const Vector2& future, const Vector2& a, const Vector2& b) {
  const Vector2 r = future - pos;
  const Vector2 s = b - a;
  const double denom = cross(r, s);
  if (std::abs(denom) < 1e-9) return -1.0;

  const double t = cross(a - pos, s) / denom;
  const double u = cross(a - pos, r) / denom;
  if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return t;
  return -1.0;
}

double computeSpeedScale(const Vector2& pos, const Vector2& driveVec, double lookaheadFactor, int& outHitEdge) {
  const Vector2 future = pos + driveVec * lookaheadFactor;
  const auto& poly = FieldConfig::FIELD_CONTOUR;
  constexpr int n = poly.size();

  double bestT = 1.0;
  bool hit = false;
  outHitEdge = -1;

  for (int i = 0; i < n; ++i) {
    if (const double t = getFirstHitT(pos, future, poly[i], poly[(i + 1) % n]); t >= 0.0 && t < bestT) {
      hit = true;
      bestT = t;
      outHitEdge = i;
    }
  }

  // Pure polygon factor: 1.0 means no hit within lookahead.
  // < 1.0 means we hit the edge, so we directly use t as the scaling factor!
  return bestT;
}

void Positioning::speedLimit(float& vx, float& vy, const Vector2& _driveVector) const {
  const double x = _cm5->getGlobalX();
  const double y = _cm5->getGlobalY();

  const Vector2 pos(x, y);

  // Convert the local drive vector to a global vector based on the robot's heading
  Vector2 globalDriveVec = _driveVector.clone();
  const double headingRad = _cm5->getHeading() * (M_PI / 180.0);
  globalDriveVec.rotate(headingRad); // Changed from -headingRad to +headingRad

  // Magic number lookahead
  constexpr double lookaheadFactor = 1.6;

  int hitEdge = -1;
  const double factor = computeSpeedScale(pos, globalDriveVec, lookaheadFactor, hitEdge);

  if (factor >= 1.0) {
    return;
  }

  constexpr float minSpeed = 20.0f;

  const auto newVx = static_cast<float>(vx * factor);
  const auto newVy = static_cast<float>(vy * factor);

  if (std::abs(vx) >= minSpeed) {
    vx = std::abs(newVx) < minSpeed ? (vx > 0 ? minSpeed : -minSpeed) : newVx;
  }
  if (std::abs(vy) >= minSpeed) {
    vy = std::abs(newVy) < minSpeed ? (vy > 0 ? minSpeed : -minSpeed) : newVy;
  }
}
