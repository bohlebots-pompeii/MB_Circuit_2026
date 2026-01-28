//
// Created by julius on 21.12.2025.
//

#ifndef BOHLEBOTS_2026_SERIAL_H
#define BOHLEBOTS_2026_SERIAL_H
#include <Arduino.h>
#include <elapsedMillis.h>

struct CalibPoint {
  float pixel;
  float cm;
};

struct Detection {
  uint8_t label;
  float bbox[4]; // x_min, y_min, x_max, y_max
  float center[2];
};

struct Object {
  uint8_t label;
  float rotation_deg;
  float dist_cm;
};

class CM5 {
public:
  void update();

  [[nodiscard]] float getHeading() const { return heading; }

  // Target goal (goal to attack)
  [[nodiscard]] float getTargetGoalRot() const { return targetGoalRot; }
  [[nodiscard]] float getTargetGoalDist() const { return targetGoalDist; }

  // Own goal (goal to defend)
  [[nodiscard]] float getOwnGoalRot() const { return ownGoalRot; }
  [[nodiscard]] float getOwnGoalDist() const { return ownGoalDist; }

  [[nodiscard]] float getBallRot() const { return ballRot; }
  [[nodiscard]] float getBallDist() const { return ballDist; }
  [[nodiscard]] bool getBallExists() const { return ballDist != 0; }

  [[nodiscard]] const Object* getObjects() const { return objects; }
  [[nodiscard]] int getNumDetections() const { return num_detections; }

  [[nodiscard]] float getGlobalX() const { return g_x; }
  [[nodiscard]] float getGlobalY() const { return g_y; }

  [[nodiscard]] bool getCM5Running() const { return lastUpdateTimer < 50;}

  [[nodiscard]] float getAwayFromOwnGoalAngle() const { return awayFromOwnGoalAngle; }

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
  float heading = 0;

  // Target goal (goal to attack) and own goal (goal to defend)
  float targetGoalRot = 0;
  float targetGoalDist = 0;
  float ownGoalRot = 0;
  float ownGoalDist = 0;

  float awayFromOwnGoalAngle = 0;

  // Goal label mapping: 1=blue, 2=yellow
  uint8_t targetGoalLabel = 2; // default: attack yellow
  uint8_t ownGoalLabel = 1;    // default: defend blue

  float ballRot = 0;
  float ballDist = 0;

  float g_x = 0;
  float g_y = 0;

  float pixelToCm(float pixel);

  float halfToFloat(uint16_t h);

  void calibMirror(const Detection* det, int num_det); // unused

  void computeCenters(Detection* det, int num_det);

  void computeRotationsAndDistances(const Detection* det, int num_det);
  void computeAwayFromOwnGoalAngle();

  void computeHeadingAndPosition(const Detection* det, int num_det);
};

#endif //BOHLEBOTS_2026_SERIAL_H