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
#include "nodes/striker/HiddenBallNPocket.h"
#include "nodes/PassBetween.h"

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

  pinMode(PINS::buttonPIN, INPUT);
}

Bot::~Bot() = default; // default

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
    // forced halt (communication module)
    halt();
    return;
  }

  if (ledTimer > 200.0 && _gameState->isRunning()) {
    // disable leds after short time so we dont confuse the enemy
    _sensors->allLEDsOff();
  }

  // ally logic
  const bool switchWanted = getSwitchWanted(ws);

  EspNow::getInstance().tick(ws, *_gameState, switchWanted);

  if (switchWanted) {
    _gameState->setRole(GameStateHandler::Role::STRIKER);
  }

  if (ws.peerSwitchWanted) {
    _gameState->setRole(GameStateHandler::Role::GOALIE);
  }

  // Action decider
  Action actionToExecute = decideAction(ws);

  switch (actionToExecute) {
  case Action::LINE_ESCAPE:
    LineEscape::execute(ws, _motion.get());
    break;
  case Action::HIDDEN_BALL_N_POCKET:
    HiddenBallNPocket::execute(ws, _motion.get());
    break;
  case Action::DRIBBLE_TO_GOAL:
    DribbleToGoal::execute(ws, _motion.get());
    break;
  case Action::GET_BEHIND_BALL:
    GetBehindBall::execute(ws, _motion.get());
    break;
  case Action::HOLD_NEUTRAL:
    HoldNeutral::execute(ws, _motion.get());
    break;
  case Action::DRIVE_TO_NEUTRAL:
    DriveToNeutral::execute(ws, _motion.get());
    break;
  case Action::INTERCEPT_BALL:
    InterceptBall::execute(ws, _motion.get());
    break;
  case Action::HALF_CIRCLE_GUARD:
    HalfCircleGuard::execute(ws, _motion.get());
    break;
  case Action::EMERGENCY_POSITION:
    EmergencyPosition::execute(ws, _motion.get());
    break;
  case Action::GOAL_NEUTRAL:
    GoalNeutral::execute(ws, _motion.get());
    break;
  }

  // kick (internal decider)
  Kick::execute(ws, _motion.get());

  if (!_gameState->isRunning()) {
    // halt if the bot state is not running. !(called after main decider for debugging)!
    halt();
    return;
  }

  // executing
  sendData(); // send data to the bottom pcb
}

Bot::Action Bot::decideAction(const WorldState& ws) {
  if (ws.lineSeen) return Action::LINE_ESCAPE;

  // Striker logic
  if (_gameState->getRole() == GameStateHandler::Role::STRIKER) {
    if (ws.hasBall && ws.hasBallTime >= GeneralConfig::HasBallValidTime) {
      return Action::HIDDEN_BALL_N_POCKET;
    }
    if (ws.ballExists && !ws.hasBall) {
      return Action::GET_BEHIND_BALL;
    }
    if (ws.lastBallSeenTime <= 500) {
      return Action::HOLD_NEUTRAL;
    }
    return Action::DRIVE_TO_NEUTRAL;
  }

  // Goalie logic
  if (!ws.ballExists && ws.peerBallValid && ws.peerAlive) {
    return Action::EMERGENCY_POSITION;
  }

  if (ws.ballExists && InterceptBall::isDrivingToBall(ws)) {
    return Action::INTERCEPT_BALL;
  }

  if (ws.ballExists && !ws.hasBall) {
    return Action::HALF_CIRCLE_GUARD;
  }

  return Action::GOAL_NEUTRAL;
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
