//
// Created by julius on 17.02.2026.
//

#ifndef BOT_2026_STRIKER_H
#define BOT_2026_STRIKER_H

#include "comms/CM5.h"
#include "Sensors.h"
#include "Positioning.h"
#include "util/Vector2.hpp"
#include <memory>

class Striker {
public:
    Striker(std::shared_ptr<CM5> cm5, std::shared_ptr<Sensors> sensors, std::shared_ptr<Positioning> positioning);

    void update() const;

private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<Positioning> _positioning;

    // Helper functions
    [[nodiscard]] Vector2 getAwayFromLineVec(int speed) const;
    [[nodiscard]] Vector2 getMoveToCenterVec(int speed) const;
    [[nodiscard]] Vector2 getBallAlignedVec(int speed) const;
    [[nodiscard]] Vector2 getBallApproachVec(int speed) const;
    [[nodiscard]] Vector2 getBallPursuitVec() const;
    [[nodiscard]] Vector2 getToNeutralPointVec() const;
    [[nodiscard]] bool checkBallOnLine() const;
    [[nodiscard]] bool checkBallInPocket() const;
};

#endif //BOT_2026_STRIKER_H

