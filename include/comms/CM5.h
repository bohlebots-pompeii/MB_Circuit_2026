//
// Created by julius on 21.12.2025.
//

#pragma once
#include <Arduino.h>
#include <elapsedMillis.h>
#include <util/Vector2.hpp>
#include <util/helper.h>

struct CalibPoint {
  float pixel;
  float cm;
};

struct Detection {
  uint8_t label;
  double bbox[4]; // x_min, y_min, x_max, y_max
  double center[2];
};

struct Object {
  uint8_t label;
  double rotation_deg;
  double dist_cm;
};

class CM5 {
public:
  void update();

  [[nodiscard]] double getHeading() const { return heading; }

  // Target goal
  [[nodiscard]] double getTargetGoalRot() const { return targetGoalRot; }
  [[nodiscard]] double getTargetGoalDist() const { return targetGoalDist; }
  [[nodiscard]] Vector2 getTargetGoalVec() const {
    if (targetGoalDist == 0) return {0, 0};
    const double angle_rad = toRad(targetGoalRot);
    return {cos(angle_rad) * targetGoalDist, sin(angle_rad) * targetGoalDist};
  }

  // Own goal
  [[nodiscard]] double getOwnGoalRot() const { return ownGoalRot; }
  [[nodiscard]] double getOwnGoalDist() const { return ownGoalDist; }
  [[nodiscard]] Vector2 getOwnGoalVec() const {
    if (ownGoalDist == 0) return {0, 0};
    const double angle_rad = toRad(ownGoalRot);
    return {cos(angle_rad) * ownGoalDist, sin(angle_rad) * ownGoalDist};
  }

  // ball
  [[nodiscard]] double getBallRot() const { return ballRot; }
  [[nodiscard]] double getBallDist() const { return ballDist; }
  [[nodiscard]] bool getBallExists() const { return ballDist != 0; }
  [[nodiscard]] Vector2 getBallVec() const {
    if (!getBallExists()) return {0, 0};
    const double angle_rad = toRad(ballRot);
    return {cos(angle_rad) * ballDist, sin(angle_rad) * ballDist};
  }

  [[nodiscard]] const Object* getObjects() const { return objects; }
  [[nodiscard]] int getNumDetections() const { return num_detections; }

  // bot position
  [[nodiscard]] double getGlobalX() const { return g_x; }
  [[nodiscard]] double getGlobalY() const { return g_y; }

  // cm5 alive
  [[nodiscard]] bool getCM5Running() const { return lastUpdateTimer < 50;}

  // helper for goalie
  [[nodiscard]] double getAwayFromOwnGoalAngle() const { return awayFromOwnGoalAngle; }

  // for target goal select
  enum COLOR {
    BLUE = 1,
    YELLOW = 2
  };

  void setTargetGoal(uint8_t goalLabel);

private:
  elapsedMillis lastUpdateTimer;

  Detection detections[6] = {};
  int num_detections = 0;
  Object objects[6] = {};
  double heading = 0;

  // Target goal (goal to attack) and own goal (goal to defend)
  double targetGoalRot = 0;
  double targetGoalDist = 0;
  double ownGoalRot = 0;
  double ownGoalDist = 0;

  double awayFromOwnGoalAngle = 0;

  // Goal label mapping: 1=blue, 2=yellow
  uint8_t targetGoalLabel = 2; // default: attack yellow
  uint8_t ownGoalLabel = 1;    // default: defend blue

  double ballRot = 0;
  double ballDist = 0;

  double g_x = 0;
  double g_y = 0;

  static double distanceFunctionBall(double x);
  static double distanceFunctionGoal(double x);

  static double halfToFloat(uint16_t h);

  static void computeCenters(Detection* det, int num_det);

  void computeRotationsAndDistances(const Detection* det, int num_det);
  void computeAwayFromOwnGoalAngle();

  void computeHeadingAndPosition(const Detection* det, int num_det);
};

