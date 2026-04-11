//
// by Julius Gerhardus on 05.01.26
//
#include <comms/CM5.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <util/helper.h>

#include "config/config.h"

constexpr double mirror_cx = 320.0;
constexpr double mirror_cy = 320.0;

double CM5::distanceFunction(const double x) {
  const double xc = std::max(97.0, std::min(x, 214.0));
  const double xn = (xc - 184.294872) / 31.994735;
  return  1.6715534999e+00 * pow(xn, 9)
        + 1.3376102010e+01 * pow(xn, 8)
        + 3.6139788395e+01 * pow(xn, 7)
        + 2.8784078503e+01 * pow(xn, 6)
        + -2.2945215557e+01 * pow(xn, 5)
        + -2.2580204450e+01 * pow(xn, 4)
        + 3.6218671956e+01 * pow(xn, 3)
        + 4.0181055143e+01 * pow(xn, 2)
        + 4.3030558229e+01 * pow(xn, 1)
        + 8.6984088433e+01;
}

// take object heights into account
double CM5::toRealLifeDistance(const double pixel, const int label) {
  const double projectedDistance = distanceFunction(pixel);
  double objectHeight = 0.0;
  if (label == 1 || label == 2) {
    objectHeight = ObjectHeights::GOAL;
  }
  else if (label == 3) {
    objectHeight = ObjectHeights::BALL;
  }

  return projectedDistance * (ObjectHeights::MIRROR - objectHeight) / (ObjectHeights::MIRROR - ObjectHeights::BALL);
}

// convert from half-float to float32 (wikipedia)
double CM5::halfToFloat(const uint16_t h) {
  uint16_t h_exp = (h & 0x7C00) >> 10;
  uint16_t h_sig = h & 0x03FF;
  const uint32_t f_sgn = (h & 0x8000) << 16;
  uint32_t f_exp, f_sig;

  if (h_exp == 0) {
    if (h_sig == 0) {
      const uint32_t f = f_sgn;
      float result;
      memcpy(&result, &f, sizeof(result));
      return result;
    }
    h_exp++;
    while ((h_sig & 0x0400) == 0) {
      h_sig <<= 1;
      h_exp--;
    }
    h_sig &= 0x03FF;
    f_exp = h_exp + (127 - 15);
    f_sig = h_sig << 13;
  } else if (h_exp == 0x1F) {
    f_exp = 255;
    f_sig = h_sig << 13;
  } else {
    f_exp = h_exp + (127 - 15);
    f_sig = h_sig << 13;
  }

  const uint32_t f = f_sgn | (f_exp << 23) | f_sig;
  float result;
  memcpy(&result, &f, sizeof(result));
  return result;
}

void CM5::computeCenters(Detection* det, const int num_det) {
  for (int i = 0; i < num_det; ++i) {
    // swapped because the camera is reverse mounted (POV: you let the hardware cook :))
    det[i].center[1] = (det[i].bbox[0] + det[i].bbox[2]) * 0.5f;
    det[i].center[0] = (det[i].bbox[1] + det[i].bbox[3]) * 0.5f;
  }
}

// compute rotation of object relative to image center
void CM5::computeRotationsAndDistances(const Detection* det, const int num_det) {
  // reset
  targetGoalDist = 0;
  targetGoalRot = 0;
  ownGoalDist = 0;
  ownGoalRot = 0;
  ballDist = 0;
  ballRot = 0;

  for (int i = 0; i < num_det; ++i) {
    // difference between img center and obj center
    const double dx = det[i].center[0] - mirror_cx;
    const double dy = det[i].center[1] - mirror_cy;

    // compute rotation of object to img center
    const double angle_rad = atan2(dy, dx);
    const auto angle_deg = toDeg(angle_rad);
    objects[i].rotation_deg =angle_deg;

    // compute distances of object from image center and estimate cm
    const double dist_px = pythagorean(dx, dy);
    //Serial.println(dist_px);
    //const double dist_cm = toRealLifeDistance(dist_px, objects[i].label);
    const double dist_cm = distanceFunction(dist_px);
    objects[i].dist_cm = dist_cm;

    objects[i].label = det[i].label;
    if (det[i].label == targetGoalLabel) {
      targetGoalRot = angle_deg;
      targetGoalDist = dist_cm;
    }
    else if (det[i].label == ownGoalLabel) {
      ownGoalRot = angle_deg;
      ownGoalDist = dist_cm;
    }
    else if (det[i].label == 3) {
      ballRot = angle_deg;
      ballDist = dist_cm;
    }
  }
}

void CM5::computeAwayFromOwnGoalAngle() {
  double awayAngle = ownGoalRot + 180.0f;
  if (awayAngle > 180.0f) awayAngle -= 360.0f;
  if (awayAngle < -180.0f) awayAngle += 360.0f;
  awayFromOwnGoalAngle = awayAngle;
}

void CM5::computeHeadingAndPosition(const Detection* det, const int num_det) {
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  bool foundOwnGoal = false, foundTargetGoal = false;

  for (int i = 0; i < num_det; ++i) {
    double theta = objects[i].rotation_deg; // deg
    theta = toRad(theta); // rad
    const double d = objects[i].dist_cm;
    const double x = d * cos(theta);
    const double y = d * sin(theta);

    if (det[i].label == ownGoalLabel) { // own goal
      x1 = x; y1 = y; foundOwnGoal = true;
    } else if (det[i].label == targetGoalLabel) { // target goal
      x2 = x; y2 = y; foundTargetGoal = true;
    }
  }

  if (foundOwnGoal && foundTargetGoal) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double h = atan2(dy, dx);
    heading = toDeg(h);

    const double mx = (x1 + x2) * 0.5f;
    const double my = (y1 + y2) * 0.5f;

    const double x = mx * cos(-h) - my * sin(-h);
    const double y = mx * sin(-h) + my * cos(-h);
    g_x = -x;
    g_y = -y;
  }
  else if (!foundOwnGoal && foundTargetGoal) {
    heading = targetGoalRot;
  }
  else {
    heading = ownGoalRot + 180.0f;
    if (heading > 180.0f) heading -= 360.0f;
    if (heading < -180.0f) heading += 360.0f;
  }
}

void CM5::update() {
  if (Serial2.available()) {
    lastUpdateTimer = 0;
    // read header
    const int num_detections_in = Serial2.read();

    if (num_detections_in == 0) {
      num_detections = 0;
      if constexpr (!GeneralConfig::DISABLE_WARNINGS) { Serial.println("WARN: No detections received."); }
      return;
    }

    const int stored_detections = (num_detections_in > 6) ? 6 : num_detections_in;
    num_detections = stored_detections;

    // read label
    for (int i = 0; i < num_detections_in; i++) {
      while (Serial2.available() < 1) {}
      const uint8_t val = Serial2.read();
      if (i < stored_detections) {
        detections[i].label = val;
      }
    }

    // read bbox data
    for (int i = 0; i < num_detections_in; i++) {
      while (Serial2.available() < 8) {}

      uint16_t raw[4];
      for (int j = 0; j < 4; j++) {
        uint8_t bytes[2];
        bytes[0] = Serial2.read();
        bytes[1] = Serial2.read();
        raw[j] = (bytes[1] << 8) | bytes[0];

        if (i < stored_detections) {
          detections[i].bbox[j] = halfToFloat(raw[j]);
        }
      }
    }

    // calibMirror(detections, stored_detections);
    computeCenters(detections, stored_detections);
    computeRotationsAndDistances(detections, stored_detections);
    computeHeadingAndPosition(detections, stored_detections);
    computeAwayFromOwnGoalAngle();
  }
}

void CM5::setTargetGoal(const uint8_t goalLabel) {
  if (goalLabel < 1 || goalLabel > 2) {
    Serial.println("ERROR: Goal label incorrect");
    return;
  }
  targetGoalLabel = goalLabel;
  ownGoalLabel = goalLabel == 1 ? 2 : 1;
}
