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


// nodes
#include <nodes/Kick.h>
#include <nodes/LineEscape.h>
#include <nodes/DriveToNeutral.h>
#include <nodes/striker/DribbleToGoal.h>
#include <nodes/striker/GetBehindBall.h>
#include <nodes/striker/HoldNeutral.h>
#include <nodes/striker/HiddenBallNPocket.h>
#include <nodes/goalie/InterceptBall.h>
#include <nodes/goalie/HalfCircleGuard.h>
#include <nodes/goalie/EmergencyPosition.h>
#include <nodes/goalie/GoalNeutral.h>

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

  elapsedMillis ledTimer;
  elapsedMillis switchWantedCooldownTimer;

  // Action bindings (function-pointer based)
  ActionLineEscape _lineEscape;
  ActionDriveToNeutral _driveToNeutral;
  ActionKick _kick;
  ActionHiddenBallNPocket _hiddenBallNPocket;
  ActionDribbleToGoal _dribbleToGoal;
  ActionGetBehindBall _getBehindBall;
  ActionHoldNeutral _holdNeutral;
  ActionInterceptBall _interceptBall;
  ActionHalfCircleGuard _halfCircleGuard;
  ActionEmergencyPosition _emergencyPosition;
  ActionGoalNeutral _goalNeutral;

  void decideAndExecute(const WorldState& ws);
  void decideKickAndExecute(const WorldState& ws);

  bool getSwitchWanted(const WorldState& ws);

  static void halt();
};
