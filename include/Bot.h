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

class Kick;

class Bot {
public:
  Bot();
  ~Bot();

  void tick();

private:
  std::shared_ptr<CM5> _cm5;
  std::shared_ptr<Sensors> _sensors;
  std::shared_ptr<Positioning> _positioning;
  std::shared_ptr<MotionController> _motion;
  std::shared_ptr<GameStateHandler> _gameState;

  std::unique_ptr<BT::BehaviorNode> _tree;
  std::unique_ptr<Kick> _kick;

  elapsedMillis ledTimer;
  elapsedMillis switchWantedCooldownTimer;

  bool getSwitchWanted(const WorldState& ws);

  static void halt();
};
