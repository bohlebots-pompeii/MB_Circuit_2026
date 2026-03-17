//
// created by Julius on 17.03.2026
//

#pragma once

#include <util/Vector2.hpp>

class CM5;
class Sensors;
class Positioning;
class GameStateHandler;

struct WorldState {
    // ball
    Vector2 ballVec;
    float ballDist;
    float ballRot;
    bool ballExists;
    bool hasBall;

    // line sensor
    bool lineSeen;
    float lineRot;
    int lineProgress;

    // position + motion
    float globalX;
    float globalY;
    float heading;
    Vector2 velocity;

    // goals
    float targetGoalRot;
    float targetGoalDist;
    Vector2 targetGoalVec;
    float ownGoalRot;
    float ownGoalDist;
    Vector2 ownGoalVec;
    float awayFromOwnGoalAngle;

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

    static WorldState build(const CM5& cm5, const Sensors& sensors, const Positioning& positioning, const GameStateHandler& gameState);
};

