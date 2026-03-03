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

// Vector helper
Vector2 getToPointVec(double x1, double y1, double x2, double y2);
bool getPointReached(double x1, double y1, double x2, double y2);

#endif //BOHLEBOTS_2026_HELPER_H