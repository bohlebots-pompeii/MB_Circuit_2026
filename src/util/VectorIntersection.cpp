//
// Created by julius on 17.02.2026.
//

#include <util/VectorIntersection.h>
#include <cmath>

void VectorIntersection::compute(const Vector2 p1, const Vector2 p2, const Vector2 p3, const Vector2 p4) {
    // Line segment 1: p1 -> p2
    // Line segment 2: p3 -> p4

    const double x1 = p1.getX();
    const double y1 = p1.getY();
    const double x2 = p2.getX();
    const double y2 = p2.getY();
    const double x3 = p3.getX();
    const double y3 = p3.getY();
    const double x4 = p4.getX();
    const double y4 = p4.getY();

    const double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // Lines are parallel
    if (std::abs(denom) < 1e-10) {
        _intersects = false;
        return;
    }

    // Calculate intersection parameters
    const double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    const double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    // Check if intersection is within both line segments
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        _intersects = true;
        _intersectionPoint.setX(x1 + t * (x2 - x1));
        _intersectionPoint.setY(y1 + t * (y2 - y1));
    } else {
        _intersects = false;
    }
}

