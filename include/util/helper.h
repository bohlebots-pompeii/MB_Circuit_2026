//
// Created by julius on 17.02.2026.
//

#ifndef BOHLEBOTS_2026_HELPER_H
#define BOHLEBOTS_2026_HELPER_H

#include <util/Vector2.hpp>

// converter
double toRad(double degrees);
double toDeg(double radians);
Vector2 degToVec(double degrees);

// solver
double pythagorean(double a, double b);
float pythagoreanf(float a, float b);

#endif //BOHLEBOTS_2026_HELPER_H