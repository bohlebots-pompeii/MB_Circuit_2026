//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H

#include "comms/CM5.h"
#include "Sensors.h"
#include <memory>
#include <Positioning.h>
#include <Vector2.hpp>

extern bool isHoming;

class Bot
{
public:
    Bot();

    void update();

    static void overrideControl();
private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<Positioning> _positioning;

    Vector2 lastLine;

    // Helper functions
    [[nodiscard]] int getRotationControl() const;
    [[nodiscard]] Vector2 getAwayFromLineVec();
    [[nodiscard]] Vector2 getMoveToCenterVec(int speed) const;
    [[nodiscard]] Vector2 getBallAlignedVec(int speed, int& rot) const;
    [[nodiscard]] Vector2 getBallApproachVec(int speed) const;
    [[nodiscard]] Vector2 getBallPursuitVec(int speed) const;
};
#endif //BOT_2026_BOT_H