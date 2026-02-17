//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H

#include "comms/CM5.h"
#include "Sensors.h"
#include <memory>
#include <Positioning.h>
#include "Striker.h"
#include "Goalie.h"

extern bool isHoming;

class Bot
{
public:
    Bot();

    void updateStriker() const;
    void updateGoalie() const;

    static void overrideControl();

    // Set which goal to attack (1=blue, 2=yellow)
    void setTargetGoal(const uint8_t goalLabel) const { _cm5->setTargetGoal(goalLabel); }

private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<Positioning> _positioning;
    std::unique_ptr<Striker> _striker;
    std::unique_ptr<Goalie> _goalie;
};
#endif //BOT_2026_BOT_H

