//
// Created by julius on 11.11.2025.
//

#include <Bot.h>
#include <Arduino.h>
#include <Wire.h>
#include <memory>
#include <motor_mb.h>
#include <comms/esp-now.h>

Bot::Bot() {
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N2, 16, 17);

    // init pointers to classes
    _cm5         = std::make_shared<CM5>();
    _sensors     = std::make_shared<Sensors>(_cm5);
    _positioning = std::make_shared<Positioning>(_cm5);
    _striker     = std::make_unique<Striker>(_cm5, _sensors, _positioning);
    _goalie      = std::make_unique<Goalie>(_cm5, _sensors, _positioning);
    _gameState   = std::make_unique<GameStateHandler>(_sensors, _cm5);

    pinMode(buttonPIN, INPUT);
}

void Bot::printPeerPacket() {
    const auto& [globalX, globalY, heading, ballRot, ballDist, flags] = espNowGetPeerData();

    Serial.println("=== Peer Packet ===");
    Serial.print("globalX:      "); Serial.println(globalX);
    Serial.print("globalY:      "); Serial.println(globalY);
    Serial.print("heading:      "); Serial.println(heading);
    Serial.print("ballRot:      "); Serial.println(ballRot);
    Serial.print("ballDist:     "); Serial.println(ballDist);
    Serial.print("running:      "); Serial.println(espNowGetFlag(flags, 0));
    Serial.print("isGoalie:     "); Serial.println(espNowGetFlag(flags, 1));
    Serial.print("seesLine:     "); Serial.println(espNowGetFlag(flags, 2));
    Serial.print("ballValid:    "); Serial.println(espNowGetFlag(flags, 3));
    Serial.print("switchWanted: "); Serial.println(espNowGetFlag(flags, 4));
    Serial.println("=== Packet End ===");
}

void Bot::update() const {
    static bool CM5_initialized = false;

    _cm5->update();
    _sensors->update();
    _positioning->update();

    Serial.println(_cm5->getGlobalX());
    Serial.println(_cm5->getGlobalY());

    EspNowPacket toSend = {};
    fillEspNowPacket(toSend);
    espNowUpdate(toSend);

    if (digitalRead(buttonPIN)) {
        pushData(false, true, 0, 0, 0, 0, false);
    }

    if (!_cm5->getCM5Running()) {
        _sensors->haltLEDs();
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    if (_cm5->getCM5Running() != CM5_initialized) {
        CM5_initialized = _cm5->getCM5Running();
        _sensors->allLEDsOff();
    }

    _gameState->update();

    if (getSwitchWantedFromPeer()) {
        _gameState->setRole(GameStateHandler::Role::GOALIE);
        Serial.println("[GAMESTATE] Switching role due to peer request");
    }

    if (Sensors::getForceHalt()) {
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    if (!_sensors->getEna()) {
        pushData(false, false, 0, 0, 0, 0, false);
        return;
    }

    if (_gameState->getRole() == GameStateHandler::Role::GOALIE) {
        _goalie->update();
    } else {
        _striker->update();
    }
}

void Bot::fillEspNowPacket(EspNowPacket& pkt) const {
    pkt.globalX  = _cm5->getGlobalX();
    pkt.globalY  = _cm5->getGlobalY();
    pkt.heading  = _cm5->getHeading();
    pkt.ballRot  = _cm5->getBallRot();
    pkt.ballDist = _cm5->getBallDist();

    const bool running   = _gameState->isRunning();
    const bool isGoalie  = _gameState->getRole() == GameStateHandler::Role::GOALIE;
    const bool seesLine  = _sensors->getLineSeen();
    const bool ballValid = _cm5->getBallExists();
    const bool switchWanted = _goalie->getSwitchWanted();

    if (switchWanted) {
        _gameState->setRole(GameStateHandler::Role::STRIKER);
    }

    pkt.flags = 0;
    espNowSetFlag(pkt.flags, 0, running);
    espNowSetFlag(pkt.flags, 1, isGoalie);
    espNowSetFlag(pkt.flags, 2, seesLine);
    espNowSetFlag(pkt.flags, 3, ballValid);
    espNowSetFlag(pkt.flags, 4, switchWanted);
}
