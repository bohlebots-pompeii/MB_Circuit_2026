//
// created by Julius on 17.03.2026
//

#pragma once

#include <util/Vector2.hpp>
#include <elapsedMillis.h>

class CM5;
class Sensors;
class Positioning;
class GameStateHandler;

struct WorldState {
  // ball
  Vector2 ballVec;
  double ballDist;
  double ballRot;
  bool ballExists;
  bool hasBall;

  // line sensor
  bool lineSeen;
  float lineRot;
  int lineProgress;

  // position + motion
  double globalX;
  double globalY;
  double heading;
  Vector2 velocity;

  // goals
  double targetGoalRot;
  double targetGoalDist;
  Vector2 targetGoalVec;
  double ownGoalRot;
  double ownGoalDist;
  Vector2 ownGoalVec;
  double awayFromOwnGoalAngle;

  // timers
  elapsedMillis lastBallSeenTime;
  elapsedMillis hasBallTime;
  elapsedMillis gameRunningTime;

  // peer robot (ESP-NOW)
  bool peerAlive;
  float peerGlobalX;
  float peerGlobalY;
  float peerHeading;
  float peerBallRot;
  float peerBallDist;
  bool peerRunning;
  bool peerIsGoalie;
  bool peerSeesLine;
  bool peerBallValid;
  bool peerSwitchWanted;

  // game state
  bool isGoalie;
  bool ena;
  bool cm5Running;

  static WorldState build(const CM5& cm5, const Sensors& sensors, const Positioning& positioning,
                          const GameStateHandler& gameState);
};
