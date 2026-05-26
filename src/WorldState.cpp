#include "WorldState.h"
#include <comms/CM5.h>
#include <Sensors.h>
#include <Positioning.h>
#include <GameStateHandler.h>
#include <comms/esp-now.h>
#include <elapsedMillis.h>
#include <config/config.h>

WorldState WorldState::build(const CM5& cm5, const Sensors& sensors, const Positioning& positioning,
                             const GameStateHandler& gameState) {
  WorldState ws;

  static elapsedMillis s_lastBallSeenTime;
  static elapsedMillis s_hasBallTime;
  static elapsedMillis s_gameRunningTime;

  // ball
  ws.ballVec = cm5.getBallVec();
  ws.ballDist = cm5.getBallDist();
  ws.ballRot = cm5.getBallRot();
  ws.ballExists = cm5.getBallExists();
  ws.hasBall = Sensors::getHasBall();

  if (ws.ballExists) {
    s_lastBallSeenTime = 0;
  }

  if (!ws.hasBall) {
    s_hasBallTime = 0;
  }

  ws.lastBallSeenTime = s_lastBallSeenTime;
  ws.hasBallTime = s_hasBallTime;

  // line sensor
  ws.lineSeen = sensors.getLineSeen();
  ws.lineRot = static_cast<float>(sensors.getLineRot());
  ws.lineProgress = sensors.getProgress();

  // position + motion
  const double rawX = cm5.getGlobalX();
  const double scaleX = FieldConfig::PERFECT_FIELD_HEIGHT / FieldConfig::REAL_FIELD_HEIGHT;
  ws.globalX = rawX * scaleX;
  //ws.globalX = cm5.getGlobalX();

  const double rawY = cm5.getGlobalY();
  const double scaleY = FieldConfig::PERFECT_FIELD_WIDTH / FieldConfig::REAL_FIELD_WIDTH;
  ws.globalY = rawY * scaleY;
  //ws.globalY = cm5.getGlobalY();

  ws.heading = cm5.getHeading();
  ws.velocity = positioning.getVelocity();

  // goals
  ws.targetGoalRot = cm5.getTargetGoalRot();
  ws.targetGoalDist = cm5.getTargetGoalDist();
  ws.targetGoalVec = cm5.getTargetGoalVec();

  ws.ownGoalRot = cm5.getOwnGoalRot();
  ws.ownGoalDist = cm5.getOwnGoalDist();
  ws.ownGoalVec = cm5.getOwnGoalVec();

  ws.awayFromOwnGoalAngle = cm5.getAwayFromOwnGoalAngle();

  ws.targetGoalTargetRot = cm5.getTargetGoalTargetRot();
  ws.targetGoalTargetDist = cm5.getTargetGoalTargetDist();
  ws.targetGoalTargetVec = cm5.getTargetGoalTargetVec();

  ws.ownGoalTargetRot = cm5.getOwnGoalTargetRot();
  ws.ownGoalTargetDist = cm5.getOwnGoalTargetDist();
  ws.ownGoalTargetVec = cm5.getOwnGoalTargetVec();

  ws.goalValid = cm5.getGoalValid();

  // peer robot (ESP-NOW)
  ws.peerAlive = espNowPeerAlive();
  if (ws.peerAlive) {
    const auto& [globalX, globalY, heading, ballRot, ballDist, flags] = espNowGetPeerData();
    ws.peerGlobalX = globalX;
    ws.peerGlobalY = globalY;
    ws.peerHeading = heading;
    ws.peerBallRot = ballRot;
    ws.peerBallDist = ballDist;
    ws.peerRunning = espNowGetFlag(flags, 0);
    ws.peerIsGoalie = espNowGetFlag(flags, 1);
    ws.peerSeesLine = espNowGetFlag(flags, 2);
    ws.peerBallValid = espNowGetFlag(flags, 3);
    ws.peerSwitchWanted = espNowGetFlag(flags, 4);
  }
  else {
    ws.peerGlobalX = 0;
    ws.peerGlobalY = 0;
    ws.peerHeading = 0;
    ws.peerBallRot = 0;
    ws.peerBallDist = 0;
    ws.peerRunning = false;
    ws.peerIsGoalie = false;
    ws.peerSeesLine = false;
    ws.peerBallValid = false;
    ws.peerSwitchWanted = false;
  }

  // derive gameRunningTime
  bool isGameRunning = false;
  if (GeneralConfig::USE_COMMUNICATION && ws.peerAlive) {
    isGameRunning = gameState.isRunning() || ws.peerRunning;
  } else {
    isGameRunning = gameState.isRunning();
  }

  if (!isGameRunning) {
    s_gameRunningTime = 0;
  }
  ws.gameRunningTime = s_gameRunningTime;

  // game state
  ws.isGoalie = gameState.getRole() == GameStateHandler::Role::GOALIE;
  ws.ena = sensors.getEna();
  ws.cm5Running = cm5.getCM5Running();

  return ws;
}
