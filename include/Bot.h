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
#include <memory>
#include <elapsedMillis.h>

class Bot {
public:
  Bot();
  ~Bot();

  enum class Action {
    LINE_ESCAPE,
    HIDDEN_BALL_N_POCKET,
    DRIBBLE_TO_GOAL,
    GET_BEHIND_BALL,
    HOLD_NEUTRAL,
    DRIVE_TO_NEUTRAL,
    INTERCEPT_BALL,
    HALF_CIRCLE_GUARD,
    EMERGENCY_POSITION,
    GOAL_NEUTRAL
  };

  void tick();

private:
  std::shared_ptr<CM5> _cm5;
  std::shared_ptr<Sensors> _sensors;
  std::shared_ptr<Positioning> _positioning;
  std::shared_ptr<MotionController> _motion;
  std::shared_ptr<GameStateHandler> _gameState;

  elapsedMillis ledTimer;
  elapsedMillis switchWantedCooldownTimer;

  bool getSwitchWanted(const WorldState& ws);

  Action decideAction(const WorldState& ws);

  static void halt();
};
