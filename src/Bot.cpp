//
// Created by julius on 11.11.2025.
//

#include <Bot.h>
#include <Arduino.h>
#include <Wire.h>
#include <memory>
#include <motor_mb.h>
#include <comms/esp-now.h>
#include <config/config.h>

// behaviour tree framework
#include <bt/PrioritySelector.h>
#include <bt/RoleSelector.h>

// nodes
#include <nodes/Kick.h>
#include <nodes/LineEscape.h>
#include <nodes/SearchMode.h>
#include <nodes/striker/DribbleToGoal.h>
#include <nodes/striker/GetBehindBall.h>
#include <nodes/striker/HoldNeutral.h>
#include <nodes/goalie/InterceptBall.h>
#include <nodes/goalie/HalfCircleGuard.h>
#include <nodes/goalie/EmergencyPosition.h>
#include <nodes/goalie/GoalNeutral.h>
#include "nodes/striker/RetrieveFromPocket.h"

Bot::Bot() {
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N2, 16, 17);

    _cm5 = std::make_shared<CM5>();
    _sensors = std::make_shared<Sensors>(_cm5);
    _positioning = std::make_shared<Positioning>(_cm5);
    _gameState = std::make_shared<GameStateHandler>(_sensors, _cm5);

    _motion = std::make_shared<MotionController>(_positioning);
    MotionController::setInstance(_motion.get());

    // build behaviour tree (action decider)
    auto striker = std::make_unique<BT::PrioritySelector>("StrikerSelector");
    striker->addChild(std::make_unique<RetrieveFromPocket>(_motion));
    striker->addChild(std::make_unique<DribbleToGoal>(_motion));
    striker->addChild(std::make_unique<GetBehindBall>(_motion));
    striker->addChild(std::make_unique<HoldNeutral>(_motion));

    auto goalie = std::make_unique<BT::PrioritySelector>("GoalieSelector");
    goalie->addChild(std::make_unique<InterceptBall>(_motion));
    goalie->addChild(std::make_unique<HalfCircleGuard>(_motion));
    goalie->addChild(std::make_unique<EmergencyPosition>(_motion));
    goalie->addChild(std::make_unique<GoalNeutral>(_motion));

    auto root = std::make_unique<BT::PrioritySelector>("RootSelector");
    root->addChild(std::make_unique<LineEscape>(_motion));
    root->addChild(std::make_unique<BT::RoleSelector>("RoleSelector", std::move(striker), std::move(goalie)));
    root->addChild(std::make_unique<SearchMode>(_motion));

    _tree = std::move(root);

    // build separate kick decider
    _kick = std::make_unique<Kick>();

    pinMode(PINS::buttonPIN, INPUT);
}

Bot::~Bot() = default; // default

void Bot::tick() {
    static bool CM5_initialized = false;

    if (digitalRead(PINS::buttonPIN)) { // kick test
        pushData(false, true, 0,0,0,0,false);
        sendData();
        return;
    }

    _cm5->update();
    _sensors->update();
    _positioning->update();

    // build world state frame
    const WorldState ws = WorldState::build(*_cm5, *_sensors, *_positioning, *_gameState);

    _motion->setRotDeltaRad(toRad(_positioning->getRotationDelta()));

    if (ledTimer > 200.0 && _gameState->isRunning()) {
        _sensors->allLEDsOff();
    }

    if (!ws.cm5Running) {
        _sensors->haltLEDs();
        halt();
        return;
    }

    if (_cm5->getCM5Running() != CM5_initialized) {
        CM5_initialized = _cm5->getCM5Running();
        _sensors->allLEDsOff();
    }

    _gameState->update();

    if (Sensors::getForceHalt()) {
        halt();
        return;
    }

    if (!ws.ena) {
        halt();
        return;
    }

    const bool switchWanted = getSwitchWanted(ws);

    EspNow::getInstance().tick(ws, *_gameState, switchWanted);

    if (switchWanted) {
        _gameState->setRole(GameStateHandler::Role::STRIKER);
    }

    if (ws.peerSwitchWanted) {
        _gameState->setRole(GameStateHandler::Role::GOALIE);
    }

    // node (action) decider
    _tree->tick(ws);

    _kick->tick(ws);

    // executing
    sendData();
}

bool Bot::getSwitchWanted(const WorldState& ws) {
    if constexpr (!GeneralConfig::USE_COMMUNICATION) {
        return false;
    }

    if (!ws.peerAlive) {
        return true;
    }

    if (ws.hasBall) {
        switchWantedCooldownTimer = 0;
        return true;
    }

    if (!ws.peerRunning) {
        switchWantedCooldownTimer = 0;
        return true;
    }

    if (!ws.ballExists) {
        return false;
    }

    if (switchWantedCooldownTimer < 2000) {
        return false;
    }

    return false;
}

void Bot::halt() {
    pushData(false, false, 0, 0, 0, 0, false);
    setKick(false);
    sendData();
}