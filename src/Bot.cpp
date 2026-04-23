//
// Created by julius on 11.11.2025.
//

// std includes
#include <Bot.h>
#include <Arduino.h>
#include <Wire.h>
#include <memory>
#include <cmath>
#include <motor_mb.h>
#include <comms/esp-now.h>
#include <config/config.h>
#include <util/PIDTuner.h>

// nodes
#include <nodes/Kick.h>
#include <nodes/LineEscape.h>
#include <nodes/DriveToNeutral.h>
#include <nodes/striker/DribbleToGoal.h>
#include <nodes/striker/GetBehindBall.h>
#include <nodes/striker/HoldNeutral.h>
#include <nodes/goalie/InterceptBall.h>
#include <nodes/goalie/HalfCircleGuard.h>
#include <nodes/goalie/EmergencyPosition.h>
#include <nodes/goalie/GoalNeutral.h>
#include <nodes/striker/HiddenBallNPocket.h>

Bot::Bot() {
  Wire.begin(); // pcb communication
  Serial.begin(115200); // debugging
  Serial2.begin(115200, SERIAL_8N2, 16, 17); // cm5 communication

  // handlers
  _cm5 = std::make_shared<CM5>();
  _sensors = std::make_shared<Sensors>(_cm5);
  _positioning = std::make_shared<Positioning>(_cm5);
  _gameState = std::make_shared<GameStateHandler>(_sensors, _cm5);

  // motion controller
  _motion = std::make_shared<MotionController>(_positioning);
  MotionController::setInstance(_motion.get()); // @TODO different solution than singleton

  pinMode(PINS::buttonPIN, INPUT); // AI PCB button pin
}

Bot::~Bot() = default;

void Bot::tick() {
  static bool CM5_initialized = false;

  if (digitalRead(PINS::buttonPIN)) {
    // kick test
    pushData(false, true, 0, 0, 0, 0, false);
    sendData();
    return;
  }

  _cm5->update();
  _sensors->update();
  _positioning->update();

  // build world state frame
  const WorldState ws = WorldState::build(*_cm5, *_sensors, *_positioning, *_gameState);

  Serial.println(ws.globalX);
  Serial.println(ws.globalY);

  // update rotation compensation for drive vec
  _motion->setRotDeltaRad(toRad(_positioning->getRotationDelta()));

  if (!ws.cm5Running) {
    // cm5 not running
    _sensors->haltLEDs();
    halt();
    return;
  }

  if (_cm5->getCM5Running() != CM5_initialized) {
    // update cm5 running
    CM5_initialized = _cm5->getCM5Running();
    _sensors->allLEDsOff();
  }

  _gameState->update();

  if (Sensors::getForceHalt()) {
    // force halt from communication module
    halt();
    return;
  }

  if (ledTimer > 200 && _gameState->isRunning()) {
    // disable leds after short time so we dont confuse the enemy
    _sensors->allLEDsOff();
  }

  // ally logic ---
  const bool switchWanted = getSwitchWanted(ws);

  EspNow::getInstance().tick(ws, *_gameState, switchWanted); // update espnow

  if (switchWanted) {
    _gameState->setRole(GameStateHandler::Role::STRIKER);
  }

  if (ws.peerSwitchWanted) {
    _gameState->setRole(GameStateHandler::Role::GOALIE);
  }
  // ally logic end ---

  // Action decider
  decideAndExecute(ws);

  // Kick decider
  decideKickAndExecute(ws);


  if (!_gameState->isRunning()) {
    // halt if the bot state is not running. !(called after main decider for debugging)!
    halt();
    return;
  }
  
  sendData(); // send data to the bottom pcb
}

void Bot::decideAndExecute(const WorldState& ws) const {
  if (ws.lineSeen) {
    _lineEscape.pFuncExec(ws, _motion.get());
    return;
  }

  // Striker logic
  if (_gameState->getRole() == GameStateHandler::Role::STRIKER) {
    if (ws.hasBall && ws.hasBallTime >= GeneralConfig::HasBallValidTime) {
      if (std::abs(ws.targetGoalRot) < 20.0) {
        _dribbleToGoal.pFuncExec(ws, _motion.get());
      }
      else {
        _hiddenBallNPocket.pFuncExec(ws, _motion.get());
      }
      return;
    }

    if (ws.ballExists && !ws.hasBall) {
      _getBehindBall.pFuncExec(ws, _motion.get());
      return;
    }

    if (ws.lastBallSeenTime <= 500) {
      _holdNeutral.pFuncExec(ws, _motion.get());
      return;
    }

    _driveToNeutral.pFuncExec(ws, _motion.get());
    return;
  }

  // Goalie logic
  if (!ws.ballExists && ws.peerBallValid && ws.peerAlive) {
    _emergencyPosition.pFuncExec(ws, _motion.get());
    return;
  }

  if (ws.ballExists && canExecuteInterceptBall(ws)) {
    _interceptBall.pFuncExec(ws, _motion.get());
    return;
  }

  if (ws.ballExists && !ws.hasBall) {
    _halfCircleGuard.pFuncExec(ws, _motion.get());
    return;
  }

  _goalNeutral.pFuncExec(ws, _motion.get());
}

void Bot::decideKickAndExecute(const WorldState& ws) const {
  // default
  setKick(false);

  // base conditions
  if (!(ws.hasBall && ws.hasBallTime >= GeneralConfig::HasBallValidTime)) {
    return;
  }

  // distance condition
  if (!(ws.targetGoalDist > 0.0 && ws.targetGoalDist < FieldConfig::kickDistance)) {
    return;
  }

  // dynamic angle condition
  const double theta = std::atan(FieldConfig::GoalSizeX / ws.targetGoalDist);

  if (const double windowDeg = toDeg(theta); !(std::abs(ws.targetGoalRot) < windowDeg)) {
    return;
  }

  // execution
  _kick.pFuncExec(ws, _motion.get());
}

bool Bot::getSwitchWanted(const WorldState& ws) {
  // decider if we want to switch roles
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

// HALT! (stop motors, no kick)
void Bot::halt() {
  pushData(false, false, 0, 0, 0, 0, false);
  setKick(false);
  sendData();
}
