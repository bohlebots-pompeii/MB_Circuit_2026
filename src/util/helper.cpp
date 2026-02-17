//
// Created by julius on 17.02.2026.
//

#include "../../include/util/helper.h"
#include <numbers>
#include <cmath>
#include <util/Vector2.hpp>

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