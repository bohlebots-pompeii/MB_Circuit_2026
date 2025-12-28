//
// Created by julius on 21.12.2025.
//

#ifndef BOHLEBOTS_2026_SERIAL_H
#define BOHLEBOTS_2026_SERIAL_H
#include <Arduino.h>

struct Object {
  uint8_t label;
  float bbox[4]; // x_min, y_min, x_max, y_max
  float center[2];
  float rotation_deg;
  float dist_cm;
  float rel_x;
  float rel_y;
};

struct CalibPoint {
  float pixel;
  float cm;
};

constexpr int MAX_DETECTIONS = 6;

extern Object public_detections[MAX_DETECTIONS];
extern int public_num_detections;

extern float heading;

static void computeCenters(Object* detections, int num_detections);

static void computeRotation(Object* detections, int num_detections);

float halfToFloat(uint16_t h);

void updateCM5();

#endif //BOHLEBOTS_2026_SERIAL_H