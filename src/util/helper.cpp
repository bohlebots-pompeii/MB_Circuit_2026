//
// Created by julius on 17.02.2026.
//

#include <util/helper.h>
#include <numbers>
#include <cmath>
#include <util/Vector2.hpp>
#include <config/config.h>

// helper
double toRad(const double degrees) {
  return degrees * (std::numbers::pi / 180.0f);
}

double toDeg(const double radians) {
  return radians * (180.0f / std::numbers::pi);
}

Vector2 degToVec(const double degrees) {
  const double radians = degrees * (std::numbers::pi / 180.0f);
  return {cos(radians), sin(radians)};
}

// solver
double pythagorean(const double a, const double b) {
  return sqrt(a * a + b * b);
}

float pythagoreanf(const float a, const float b) {
  return sqrtf(a * a + b * b);
}

// Vector helper
Vector2 getToPointVec(const double x1, const double y1, const double x2, const double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;

  return Vector2(dx, dy);
}

bool getPointReached(const double x1, const double y1, const double x2, const double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;

  const bool x_reached = std::abs(dx) < FieldConfig::POINT_REACHED_DIST;
  const bool y_reached = std::abs(dy) < FieldConfig::POINT_REACHED_DIST;

  if (x_reached && y_reached) {
    return true;
  }

  return false;
}
