//
// Created by julius on 21.12.2025.
//

#ifndef BOHLEBOTS_2026_SERIAL_H
#define BOHLEBOTS_2026_SERIAL_H
#include <Arduino.h>

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
  float rel_x;
  float rel_y;
};

class CM5 {
public:
  void update();

  [[nodiscard]] float getHeading() const;

  [[nodiscard]] float getYellowRot() const { return yellowRot; }
  [[nodiscard]] float getYellowDist() const { return yellowDist; }

  [[nodiscard]] float getBlueRot() const { return blueRot; }
  [[nodiscard]] float getBlueDist() const { return blueDist; }

  [[nodiscard]] float getBallRot() const { return ballRot; }
  [[nodiscard]] float getBallDist() const { return ballDist; }

  [[nodiscard]] const Object* getObjects() const { return objects; }
  [[nodiscard]] int getNumDetections() const { return num_detections; }

private:
  Detection detections[6] = {};
  int num_detections = 0;
  Object objects[6] = {};
  float heading = 0;

  int16_t yellowRot = 0;
  int16_t yellowDist = 0;

  int16_t blueRot = 0;
  int16_t blueDist = 0;

  int16_t ballRot = 0;
  int16_t ballDist = 0;

  float pixelToCm(float pixel);

  float halfToFloat(uint16_t h);

  void calibMirror(const Detection* det, int num_det); // unused

  void computeCenters(Detection* det, int num_det);

  void computeRotations(const Detection* det, int num_det);

  void computeDistances(const Detection* det, int num_det);

  void computeHeading(int num_det);

  void computeHeadingFromPolar(const Detection* det, int num_det);
};

#endif //BOHLEBOTS_2026_SERIAL_H