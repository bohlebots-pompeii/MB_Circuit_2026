//
// Created by julius on 22.02.2026.
//

#include "../include/GameStateHandler.h"
#include <Arduino.h>
#include "util/GameStateStore.h"

GameStateHandler::GameStateHandler(std::shared_ptr<Sensors> sensors, std::shared_ptr<CM5> cm5)
    : _sensors(std::move(sensors)), _cm5(std::move(cm5)) {

    const auto [valid, role, targetGoal] = GameStateStore::load();
    if (valid) {
        _role           = role    ? Role::STRIKER : Role::GOALIE;
        _targetIsYellow = targetGoal != 0;
        _cm5->setTargetGoal(_targetIsYellow ? 2 : 1);
        _sensors->setEna(true);
        _state              = State::RUNNING;
        _restoredFromEeprom = true;
        Serial.println("[GameState] Resumed from EEPROM.");
    } else {
        _cm5->setTargetGoal(1);
    }

    applyTargetLED();
}

void GameStateHandler::update() {
    switch (_state) {
        case State::TARGET_SELECT: handleTargetSelect(); break;
        case State::LOCKED:        handleLocked();        break;
        case State::ROLE_SELECT:   handleRoleSelect();    break;
        case State::RUNNING:       handleRunning();       break;
    }
}

void GameStateHandler::handleTargetSelect() {
    const bool left  = _sensors->getButtonState(0, 1);
    const bool right = _sensors->getButtonState(0, 2);

    if (right && !_lastRight) {
        _targetIsYellow = !_targetIsYellow;
        _cm5->setTargetGoal(_targetIsYellow ? 2 : 1);
        applyTargetLED();
    }

    if (left && !_lastLeft) {
        _blinkCount  = 0;
        _blinkLedOn  = false;
        _blinkLastMs = millis();
        _state = State::LOCKED;
    }

    _lastLeft  = left;
    _lastRight = right;
}

void GameStateHandler::handleLocked() {
    if (const uint32_t now = millis(); now - _blinkLastMs >= BLINK_INTERVAL_MS) {
        _blinkLastMs = now;
        _blinkLedOn  = !_blinkLedOn;
        _sensors->setLED(0, 1, _blinkLedOn ? targetColor() : Sensors::OFF);
        _sensors->setLED(0, 2, _blinkLedOn ? targetColor() : Sensors::OFF);
        if (!_blinkLedOn) {
            _blinkCount++;
        }
    }

    if (_blinkCount >= BLINK_TIMES) {
        _sensors->allLEDsOff();
        _state = State::ROLE_SELECT;
        applyRoleLED();
        _lastLeft  = _sensors->getButtonState(0, 1);
        _lastRight = _sensors->getButtonState(0, 2);
    }
}

void GameStateHandler::handleRoleSelect() {
    const bool left  = _sensors->getButtonState(0, 1);
    const bool right = _sensors->getButtonState(0, 2);

    if (left && !_lastLeft) {
        _role = (_role == Role::GOALIE) ? Role::STRIKER : Role::GOALIE;
        applyRoleLED();
    }

    if (right && !_lastRight) {
        _sensors->setEna(true);
        _state = State::RUNNING;
        applyRoleLED();
        saveToEeprom();  // persist so we survive a power blip
    }

    _lastLeft  = left;
    _lastRight = right;
}

void GameStateHandler::handleRunning() {
    const bool left  = _sensors->getButtonState(0, 1);
    const bool right = _sensors->getButtonState(0, 2);

    // Either button stops the bot → back to role select
    if ((left && !_lastLeft) || (right && !_lastRight)) {
        _sensors->setEna(false);
        _state = State::ROLE_SELECT;
        _sensors->allLEDsOff();
        applyRoleLED();
        GameStateStore::clear();  // deliberate stop – don't auto-resume
        _lastLeft  = left;
        _lastRight = right;
        return;
    }

    _lastLeft  = left;
    _lastRight = right;
}

// helpers
void GameStateHandler::applyRoleLED() const {
    _sensors->setLED(0, 1, _role == Role::GOALIE ? Sensors::GREEN : Sensors::CYAN);
}

void GameStateHandler::applyTargetLED() const {
    _sensors->setLED(0, 2, targetColor());
}

void GameStateHandler::saveToEeprom() const {
    const uint8_t roleVal   = (_role == Role::STRIKER) ? 1 : 0;
    const uint8_t targetVal = _targetIsYellow ? 1 : 0;
    GameStateStore::save(roleVal, targetVal);
}

