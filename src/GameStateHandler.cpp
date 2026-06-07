//
// Created by julius on 22.02.2026.
//

#include <GameStateHandler.h>
#include <Arduino.h>
#include "util/GameStateStore.h"

GameStateHandler::GameStateHandler(std::shared_ptr<Sensors> sensors, std::shared_ptr<CM5> cm5)
  : _sensors(std::move(sensors)), _cm5(std::move(cm5)) {
  if (const auto [valid, role, targetGoal] = GameStateStore::load(); valid) {
    _role = role ? Role::STRIKER : Role::GOALIE;
    _targetIsYellow = targetGoal != 0;
    _cm5->setTargetGoal(_targetIsYellow ? 2 : 1);
    _sensors->setEna(true);
    _state = State::RUNNING;
    _restoredFromEeprom = true;
    Serial.println("[GameState] Resumed from EEPROM.");
  }
  else {
    _cm5->setTargetGoal(1);
  }

  applyTargetLED();
}

void GameStateHandler::update() {
  switch (_state) {
  case State::TARGET_SELECT: handleTargetSelect();
    break;
  case State::LOCKED: handleLocked();
    break;
  case State::ROLE_SELECT: handleRoleSelect();
    break;
  case State::RUNNING: handleRunning();
    break;
  }
}

void GameStateHandler::handleTargetSelect() {
  const bool left = _sensors->getButtonState(0, 1);
  const bool right = _sensors->getButtonState(0, 2);
  const bool forceHalt = Sensors::getForceHalt();

  if (right && !_lastRight) {
    _targetIsYellow = !_targetIsYellow;
    _cm5->setTargetGoal(_targetIsYellow ? 2 : 1);
    applyTargetLED();
  }

  if (left && !_lastLeft) {
    _blinkCount = 0;
    _blinkLedOn = false;
    _blinkLastMs = millis();
    _state = State::LOCKED;
  }

  _lastLeft = left;
  _lastRight = right;
  _lastForceHalt = forceHalt;
}

void GameStateHandler::handleLocked() {
  const bool forceHalt = Sensors::getForceHalt();

  if (const uint32_t now = millis(); now - _blinkLastMs >= BLINK_INTERVAL_MS) {
    _blinkLastMs = now;
    _blinkLedOn = !_blinkLedOn;
    _sensors->setLED(0, 1, _blinkLedOn ? targetColor() : Sensors::OFF);
    _sensors->setLED(0, 2, _blinkLedOn ? targetColor() : Sensors::OFF);
    if (!_blinkLedOn) {
      _blinkCount++;
    }
  }

  if (_blinkCount >= BLINK_TIMES) {
    _sensors->allLEDsOff();
    _state = State::ROLE_SELECT;
    _stateEnterMs = millis();
    applyRoleLED();
    _lastLeft = _sensors->getButtonState(0, 1);
    _lastRight = _sensors->getButtonState(0, 2);
  }

  _lastForceHalt = forceHalt;
}

void GameStateHandler::handleRoleSelect() {
  const bool left = _sensors->getButtonState(0, 1);
  const bool right = _sensors->getButtonState(0, 2);
  const bool forceHalt = Sensors::getForceHalt();

  if (left && !_lastLeft) {
    _role = (_role == Role::GOALIE) ? Role::STRIKER : Role::GOALIE;
    applyRoleLED();
  }

  // Right button starts, OR comms module START (forceHalt going FALSE)
  if ((right && !_lastRight) || (!forceHalt && _lastForceHalt)) {
    Serial.println("[GameState] Transitioning to RUNNING!");
    _sensors->setEna(true);
    _state = State::RUNNING;
    _stateEnterMs = millis();
    applyRoleLED();
    saveToEeprom(); // persist so we survive a power blip
  }

  _lastLeft = left;
  _lastRight = right;
  _lastForceHalt = forceHalt;
}

void GameStateHandler::handleRunning() {
  const bool left = _sensors->getButtonState(0, 1);
  const bool right = _sensors->getButtonState(0, 2);
  const bool forceHalt = Sensors::getForceHalt();

  // Protect against severe voltage dips when motors activate causing immediate false stops
  if (millis() - _stateEnterMs < 250) {
    _lastLeft = left;
    _lastRight = right;
    _lastForceHalt = forceHalt;
    return;
  }

  // Either button stops the bot → back to role select, OR comms module STOP (forceHalt going TRUE)
  if ((left && !_lastLeft) || (right && !_lastRight) || (forceHalt && !_lastForceHalt)) {
    Serial.println("[GameState] Transitioning back to ROLE_SELECT!");
    _sensors->setEna(false);
    _state = State::ROLE_SELECT;
    _stateEnterMs = millis();
    _sensors->allLEDsOff();
    applyRoleLED();
    GameStateStore::clear(); // clear EEPROM for no autoresume
    _lastLeft = left;
    _lastRight = right;
    _lastForceHalt = forceHalt;
    return;
  }

  _lastLeft = left;
  _lastRight = right;
  _lastForceHalt = forceHalt;
}

// helpers
void GameStateHandler::applyRoleLED() const {
  _sensors->setLED(0, 1, _role == Role::GOALIE ? Sensors::GREEN : Sensors::CYAN);
}

void GameStateHandler::applyTargetLED() const {
  _sensors->setLED(0, 2, targetColor());
}

void GameStateHandler::saveToEeprom() const {
  const uint8_t roleVal = (_role == Role::STRIKER) ? 1 : 0;
  const uint8_t targetVal = _targetIsYellow ? 1 : 0;
  GameStateStore::save(roleVal, targetVal);
}
