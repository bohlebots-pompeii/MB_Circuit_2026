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
#include "GameStateHandler.h"

extern bool isHoming;

class Bot {
public:
    Bot();

    // Main loop entry point – dispatches to correct role
    void update() const;

    static void overrideControl();

private:
    std::shared_ptr<CM5>          _cm5;
    std::shared_ptr<Sensors>      _sensors;
    std::shared_ptr<Positioning>  _positioning;
    std::unique_ptr<Striker>      _striker;
    std::unique_ptr<Goalie>       _goalie;
    std::unique_ptr<GameStateHandler> _gameState;
};

#endif //BOT_2026_BOT_H

