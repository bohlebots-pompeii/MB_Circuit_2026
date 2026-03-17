//
// Created by julius on 11.11.2025.
//

#pragma once

#include "comms/CM5.h"
#include "Sensors.h"
#include "Positioning.h"
#include "GameStateHandler.h"
#include "MotionController.h"
#include "WorldState.h"
#include <bt/BehaviorNode.h>
#include <memory>
#include <elapsedMillis.h>

class Bot {
public:
    Bot();

    void update();

private:
    std::shared_ptr<CM5>              _cm5;
    std::shared_ptr<Sensors>          _sensors;
    std::shared_ptr<Positioning>      _positioning;
    std::shared_ptr<MotionController> _motion;
    std::unique_ptr<GameStateHandler> _gameState;

    std::unique_ptr<BT::BehaviorNode> _tree;

    elapsedMillis ledTimer;
    elapsedMillis switchWantedCooldownTimer;

    // Helper for getSwitchWanted logic (ported from Goalie)
    bool getSwitchWanted(const WorldState& ws);
};

