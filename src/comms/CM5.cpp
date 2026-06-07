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

static constexpr uint8_t BALL_LABEL = 3;
static constexpr int MAX_DETECTIONS = 6;
static constexpr uint32_t SERIAL_TIMEOUT_MS = 50;

double CM5::distanceFunctionBall(const double x) {
  if (x < 70.0) return 10.0;
  if (x > 235.0) return 250.0;

  return -2.6935371893632066e+02
    + x * (1.1973576303885302e+01
      + x * (-2.0428550504472429e-01
        + x * (1.7299131498039267e-03
          + x * (-7.0957614492572805e-06
            + x * 1.1411888587287121e-08))));
}

double CM5::distanceFunctionGoal(const double x) {
  if (x < 78.0) return 10.0;
  if (x > 242.0) return 225.0;

  return -4.7116022519053649e+02
    + x * (2.0509237406625694e+01
      + x * (-3.4042589603176654e-01
        + x * (2.7337786762314385e-03
          + x * (-1.0535352217327996e-05
            + x * 1.5757400667181950e-08))));
}

double CM5::halfToFloat(const uint16_t h) {
  int32_t h_exp = (h & 0x7C00) >> 10;
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
    f_exp = static_cast<uint32_t>(h_exp + (127 - 15));
    f_sig = h_sig << 13;
  }
  else if (h_exp == 0x1F) {
    f_exp = 255;
    f_sig = h_sig << 13;
  }
  else {
    f_exp = static_cast<uint32_t>(h_exp + (127 - 15));
    f_sig = h_sig << 13;
  }

  const uint32_t f = f_sgn | (f_exp << 23) | f_sig;
  // we need to keep the parentheses to force the compiler to keep them for saftey
  float result;
  memcpy(&result, &f, sizeof(result));
  return result;
}

void CM5::computeCenters(Detection* det, const int num_det) {
  for (int i = 0; i < num_det; ++i) {
    // we need to swap centers because the camera is reverse mounted (POV: you let the hardware cook ^^)
    det[i].center[1] = (det[i].bbox[0] + det[i].bbox[2]) * 0.5;
    det[i].center[0] = (det[i].bbox[1] + det[i].bbox[3]) * 0.5;
  }
}

void CM5::computeRotationsAndDistances(const Detection* det, const int num_det) {
  targetGoalDist = 0;
  targetGoalRot = 0;
  ownGoalDist = 0;
  ownGoalRot = 0;
  targetGoalTargetDist = 0;
  targetGoalTargetRot = 0;
  ownGoalTargetDist = 0;
  ownGoalTargetRot = 0;
  ballDist = 0;
  ballRot = 0;

  for (int i = 0; i < num_det; ++i) {
    objects[i].label = det[i].label;
    const double dx = det[i].center[0] - mirror_cx;
    const double dy = det[i].center[1] - mirror_cy;

    const double angle_rad = std::atan2(dy, dx);
    const auto angle_deg = toDeg(angle_rad);
    objects[i].rotation_deg = angle_deg;

    const double dist_px = std::hypot(dx, dy);
    double dist_cm;
    if (objects[i].label == BALL_LABEL) {
      dist_cm = distanceFunctionBall(dist_px);
    }
    else {
      dist_cm = distanceFunctionGoal(dist_px);
    }
    objects[i].dist_cm = dist_cm;

    if (Objects_DEBUG) {
      Serial.print("Obj[");
      Serial.print(i);
      Serial.print("] L:");
      Serial.print(objects[i].label);
      Serial.print(" rot:");
      Serial.print(objects[i].rotation_deg);
      Serial.print(" d_px:");
      Serial.print(dist_px);
      Serial.print(" d_cm:");
      Serial.println(objects[i].dist_cm);
    }

    if (det[i].label == targetGoalLabel) {
      targetGoalRot = angle_deg;
      targetGoalDist = dist_cm;
    }
    else if (det[i].label == ownGoalLabel) {
      ownGoalRot = angle_deg;
      ownGoalDist = dist_cm;
    }
    else if (det[i].label == targetGoalTargetLabel) {
      targetGoalTargetRot = angle_deg;
      targetGoalTargetDist = dist_cm;
    }
    else if (det[i].label == ownGoalTargetLabel) {
      ownGoalTargetRot = angle_deg;
      ownGoalTargetDist = dist_cm;
    }
    else if (det[i].label == BALL_LABEL) {
      ballRot = angle_deg;
      ballDist = dist_cm;
    }
  }
}

void CM5::computeAwayFromOwnGoalAngle() {
  const double angle = ownGoalRot;
  if (ownGoalDist == 0 && targetGoalDist != 0) {
    awayFromOwnGoalAngle = targetGoalRot;
    return;
  }
  double awayAngle = angle + 180.0;
  if (awayAngle > 180.0) awayAngle -= 360.0;
  if (awayAngle < -180.0) awayAngle += 360.0;
  awayFromOwnGoalAngle = awayAngle;
}

void CM5::computeHeadingAndPosition(const Detection* det, const int num_det) {
  double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
  bool foundOwnGoal = false, foundTargetGoal = false;
  goalValid = true;

  for (int i = 0; i < num_det; ++i) {
    const double theta = toRad(objects[i].rotation_deg);
    const double d = objects[i].dist_cm;
    const double x = d * std::cos(theta);
    const double y = d * std::sin(theta);

    if (det[i].label == ownGoalLabel) {
      x1 = x;
      y1 = y;
      foundOwnGoal = true;
    }
    else if (det[i].label == targetGoalLabel) {
      x2 = x;
      y2 = y;
      foundTargetGoal = true;
    }
  }

  if (foundOwnGoal && foundTargetGoal) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double h = std::atan2(dy, dx);
    heading = toDeg(h);

    const double mx = (x1 + x2) * 0.5;
    const double my = (y1 + y2) * 0.5;

    auto position = Vector2(mx, my);
    position.rotate(-h);
    g_x = -position.getX();
    g_y = -position.getY();
  }
  else if (!foundOwnGoal && foundTargetGoal) {
    heading = targetGoalRot;
  }
  else if (foundOwnGoal) {
    heading = awayFromOwnGoalAngle;
  }
  else {
    goalValid = false;
    heading = 0.0;
  }
}

void CM5::update() {
  if (Serial2.available()) {
    lastUpdateTimer = 0;
    const int num_detections_in = Serial2.read();

    if (num_detections_in <= 0 || num_detections_in > MAX_DETECTIONS) {
      num_detections = 0;
      if constexpr (!GeneralConfig::DISABLE_WARNINGS) { Serial.println("WARN: No detections received."); }
      return;
    }

    const int stored_detections = std::min(num_detections_in, MAX_DETECTIONS);
    num_detections = stored_detections;

    // Read all labels first
    for (int i = 0; i < num_detections_in; i++) {
      elapsedMillis timeout;
      while (Serial2.available() < 1) {
        if (timeout > SERIAL_TIMEOUT_MS) {
          num_detections = 0;
          return;
        }
      }
      const uint8_t val = Serial2.read();
      if (i < stored_detections) {
        detections[i].label = val;
      }
    }

    // Then read all bboxes
    for (int i = 0; i < num_detections_in; i++) {
      elapsedMillis timeout;
      while (Serial2.available() < 8) {
        if (timeout > SERIAL_TIMEOUT_MS) {
          num_detections = 0;
          return;
        }
      }

      for (double& j : detections[i].bbox) {
        uint8_t bytes[2];
        bytes[0] = Serial2.read();
        bytes[1] = Serial2.read();
        const uint16_t raw = (bytes[1] << 8) | bytes[0];
        if (i < stored_detections) {
          j = halfToFloat(raw);
        }
      }
    }

    computeCenters(detections, stored_detections);
    computeRotationsAndDistances(detections, stored_detections);
    computeAwayFromOwnGoalAngle();
    computeHeadingAndPosition(detections, stored_detections);
  }
}

void CM5::setTargetGoal(const uint8_t goalLabel) {
  if (goalLabel < 1 || goalLabel > 2) {
    Serial.println("ERROR: Goal label incorrect");
    return;
  }
  if (goalLabel == 1) {
    // 1 = Blue
    targetGoalLabel = 1; // blue goal
    targetGoalTargetLabel = 4; // blue target
    ownGoalLabel = 2; // yellow goal
    ownGoalTargetLabel = 5; // yellow target
  }
  else {
    // 2 = Yellow
    targetGoalLabel = 2; // yellow goal
    targetGoalTargetLabel = 5; // yellow target
    ownGoalLabel = 1; // blue goal
    ownGoalTargetLabel = 4; // blue target
  }
}
