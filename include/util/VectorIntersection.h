//
// Created by julius on 17.02.2026.
//

#ifndef BOT_2026_VECTORINTERSECTION_H
#define BOT_2026_VECTORINTERSECTION_H

#include <util/Vector2.hpp>

class VectorIntersection {
public:
    VectorIntersection() = default;

    // Compute intersection between two line segments
    void compute(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4);

    [[nodiscard]] bool intersects() const { return _intersects; }
    [[nodiscard]] Vector2 getIntersectionPoint() const { return _intersectionPoint; }

private:
    bool _intersects = false;
    Vector2 _intersectionPoint{0, 0};
};

#endif //BOT_2026_VECTORINTERSECTION_H

