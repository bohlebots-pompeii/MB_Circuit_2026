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

#include <bt/PrioritySelector.h>
#include <bt/RoleSelector.h>

#include <nodes/Kick.h>
#include <nodes/LineEscape.h>
#include <nodes/SearchMode.h>
#include <nodes/striker/DribbleToGoal.h>
#include <nodes/striker/GetBehinBall.h>
#include <nodes/striker/HoldNeutral.h>
#include <nodes/goalie/InterceptBall.h>
#include <nodes/goalie/HalfCircleGuard.h>
#include <nodes/goalie/EmergencyPosition.h>
#include <nodes/goalie/GoalNeutral.h>

Bot::Bot() {
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N2, 16, 17);

    _cm5 = std::make_shared<CM5>();
    _sensors = std::make_shared<Sensors>(_cm5);
    _positioning = std::make_shared<Positioning>(_cm5);
    _gameState = std::make_unique<GameStateHandler>(_sensors, _cm5);

    _motion = std::make_shared<MotionController>();
    MotionController::setInstance(_motion.get());

    // build behaviour tree
    auto striker = std::make_unique<BT::PrioritySelector>();
    striker->addChild(std::make_unique<DribbleToGoal>(_motion));
    striker->addChild(std::make_unique<GetBehinBall>(_motion));
    striker->addChild(std::make_unique<HoldNeutral>(_motion));

    auto goalie = std::make_unique<BT::PrioritySelector>();
    goalie->addChild(std::make_unique<InterceptBall>(_motion));
    goalie->addChild(std::make_unique<HalfCircleGuard>(_motion));
    goalie->addChild(std::make_unique<EmergencyPosition>(_motion));
    goalie->addChild(std::make_unique<GoalNeutral>(_motion));

    auto root = std::make_unique<BT::PrioritySelector>();
    root->addChild(std::make_unique<LineEscape>(_motion));
    root->addChild(std::make_unique<Kick>(_motion));
    root->addChild(std::make_unique<BT::RoleSelector>(std::move(striker), std::move(goalie)));
    root->addChild(std::make_unique<SearchMode>(_motion));

    _tree = std::move(root);

    pinMode(buttonPIN, INPUT);
}

void Bot::update() {
    static bool CM5_initialized = false;

    _cm5->update();
    _sensors->update();
    _positioning->update();

    // build world state frame
    const WorldState ws = WorldState::build(*_cm5, *_sensors, *_positioning, *_gameState);

    _motion->setRotDeltaRad(toRad(_positioning->getRotationDelta()));

    if (ledTimer > 200 && _gameState->isRunning()) {
        _sensors->allLEDsOff();
    }

    const bool switchWanted = getSwitchWanted(ws);

    if (switchWanted) {
        _gameState->setRole(GameStateHandler::Role::STRIKER);
    }

    EspNowPacket toSend = {};
    toSend.globalX = ws.globalX;
    toSend.globalY = ws.globalY;
    toSend.heading = ws.heading;
    toSend.ballRot = ws.ballRot;
    toSend.ballDist = ws.ballDist;

    const bool currentIsGoalie = _gameState->getRole() == GameStateHandler::Role::GOALIE;
    const bool running = _gameState->isRunning();

    toSend.flags = 0;
    espNowSetFlag(toSend.flags, 0, running);
    espNowSetFlag(toSend.flags, 1, currentIsGoalie);
    espNowSetFlag(toSend.flags, 2, ws.lineSeen);
    espNowSetFlag(toSend.flags, 3, ws.ballExists);
    espNowSetFlag(toSend.flags, 4, switchWanted);

    espNowUpdate(toSend);

    if (digitalRead(buttonPIN)) {
        pushData(false, true, 0, 0, 0, 0, false);
    }

    if (!ws.cm5Running) {
        _sensors->haltLEDs();
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    if (_cm5->getCM5Running() != CM5_initialized) {
        CM5_initialized = _cm5->getCM5Running();
        _sensors->allLEDsOff();
    }

    _gameState->update();

    if (ws.peerSwitchWanted) {
        _gameState->setRole(GameStateHandler::Role::GOALIE);
    }

    if (Sensors::getForceHalt()) {
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    if (!ws.ena) {
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    // update normal behaviour tree
    _tree->tick(ws);
}

bool Bot::getSwitchWanted(const WorldState& ws) {
    if constexpr (!USE_COMMUNICATION) {
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
