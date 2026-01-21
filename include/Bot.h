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
    int getRotationControl() const;
    Vector2 getAwayFromLineVec();
    Vector2 getMoveToCenterVec(int speed) const;
    Vector2 getBallAlignedVec(int speed, int& rot);
    Vector2 getBallApproachVec(int speed) const;
    Vector2 getBallPursuitVec(int speed) const;
};
#endif //BOT_2026_BOT_H

