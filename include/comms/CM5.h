//
// Created by julius on 21.12.2025.
//

#ifndef BOHLEBOTS_2026_SERIAL_H
#define BOHLEBOTS_2026_SERIAL_H
#include <Arduino.h>
#include <numbers>

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

  [[nodiscard]] float getYellowRot() const { return yellowRot; }
  [[nodiscard]] float getYellowDist() const { return yellowDist; }

  [[nodiscard]] float getBlueRot() const { return blueRot; }
  [[nodiscard]] float getBlueDist() const { return blueDist; }

  [[nodiscard]] float getBallRot() const { return ballRot; }
  [[nodiscard]] float getBallDist() const { return ballDist; }
  [[nodiscard]] bool getBallExists() const { return ballDist != 0; }

  [[nodiscard]] const Object* getObjects() const { return objects; }
  [[nodiscard]] int getNumDetections() const { return num_detections; }

  [[nodiscard]] float getGlobalX() const { return g_x; }
  [[nodiscard]] float getGlobalY() const { return g_y; }

private:
  Detection detections[6] = {};
  int num_detections = 0;
  Object objects[6] = {};
  float heading = 0;

  float yellowRot = 0;
  float yellowDist = 0;

  float blueRot = 0;
  float blueDist = 0;

  float ballRot = 0;
  float ballDist = 0;

  float g_x = 0;
  float g_y = 0;

  float pixelToCm(float pixel);

  float halfToFloat(uint16_t h);

  void calibMirror(const Detection* det, int num_det); // unused

  void computeCenters(Detection* det, int num_det);

  void computeRotationsAndDistances(const Detection* det, int num_det);

  void computeHeadingAndPosition(const Detection* det, int num_det);
};

#endif //BOHLEBOTS_2026_SERIAL_H