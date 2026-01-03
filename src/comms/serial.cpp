// cpp
#include <comms/serial.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Object public_detections[MAX_DETECTIONS]; // memory because memory issues
int public_num_detections = 0;
float heading = 0.0f;

constexpr CalibPoint calib[] = {
  {  81.0f,  10.0f },
  { 125.0f,  20.0f },
  { 147.0f,  30.0f },
  { 160.0f,  40.0f },
  { 174.0f,  50.0f },
  { 181.7f, 60.0f },
  { 189.9f, 70.0f },
  { 197.2f, 80.0f },
  { 201.4f, 90.0f },
  { 206.0f,100.0f },
  { 209.0f,110.0f },
  { 210.0f,120.0f },
  { 211.5f,130.0f },
  { 211.0f,140.0f },
  { 216.5f,150.0f },
  { 217.0f,160.0f },
  { 219.5f,170.0f },
  { 221.0f,180.0f },
  { 223.0f,190.0f },
  { 224.0f,200.0f },
  { 227.15f,210.0f },
  { 229.0f,220.0f }
};

constexpr int CALIB_N = std::size(calib);

static float pixelToCm(const float pixel) {
  if (pixel <= calib[0].pixel) return calib[0].cm;
  if (pixel >= calib[CALIB_N - 1].pixel) return calib[CALIB_N - 1].cm;

  for (int i = 0; i < CALIB_N - 1; i++) {
    if (pixel >= calib[i].pixel && pixel <= calib[i + 1].pixel) {
      const float t = (pixel - calib[i].pixel) /
                (calib[i + 1].pixel - calib[i].pixel);
      return calib[i].cm +
             t * (calib[i + 1].cm - calib[i].cm);
    }
  }
  return 0.0f;
}

void calibMirror(const Object* detections, const int num_detections) {
  constexpr float cx = 320.0f; // screen center
  constexpr float cy = 320.0f;
  for (int i = 0; i < num_detections; ++i) {
    if (detections[i].label == 1 || detections[i].label == 2) {
      const float dx = detections[i].center[0] - cx;
      const float dy = detections[i].center[1] - cy;

      const float r = sqrtf(dx * dx + dy * dy);
      Serial.println(r);
    }
  }
}

static void computeCenters(Object* detections, const int num_detections) {
  for (int i = 0; i < num_detections; ++i) {
    detections[i].center[0] = (detections[i].bbox[0] + detections[i].bbox[2]) * 0.5f;
    detections[i].center[1] = (detections[i].bbox[1] + detections[i].bbox[3]) * 0.5f;
  }
}

float halfToFloat(const uint16_t h) {
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

static void computeRotations(Object* detections, const int num_detections) {
  for (int i = 0; i < num_detections; ++i) {
    constexpr float cx = 320.0f; // screen center
    constexpr float cy = 320.0f;
    const float dx = detections[i].center[0] - cx; // position relative to center
    const float dy = detections[i].center[1] - cy;
    const float angle_rad = atan2f(dx, dy); // rotation relative to center
    detections[i].rotation_deg = angle_rad * 180.0f / M_PI; // to deg
  }
}

void computeDistances(Object* detections, const int num_detections) {
  constexpr float cx = 320.0f; // mirror center
  constexpr float cy = 320.0f;

  for (int i = 0; i < num_detections; ++i) {
    const float dx = detections[i].center[0] - cx;
    const float dy = detections[i].center[1] - cy;

    const float r = sqrtf(dx * dx + dy * dy);

    const float dist = pixelToCm(r);
    detections[i].dist_cm = dist;

    const float angle_rad = atan2f(dx, dy);

    // position rel to robot
    detections[i].rel_x = dist * cosf(angle_rad);
    detections[i].rel_y = dist * sinf(angle_rad);
  }
}

static void computeHeading(const Object* detections, const int num_detections) {
  bool blue = false;
  bool yellow = false;
  float bluerot = 0.0f, yellowrot = 0.0f;
  for (int i = 0; i < num_detections; ++i) {
    if (detections[i].label == 1) {
      blue = true;
      bluerot = detections[i].rotation_deg;
    }
    else if (detections[i].label == 2) {
      yellow = true;
      yellowrot = detections[i].rotation_deg;
    }
  }
  if ((!blue && yellow) || (blue && !yellow)) { return; }
  heading = yellowrot - bluerot;
}

void computeHeadingFromPolar(const Object* detections, const int num_detections) {
  float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  bool foundBlue = false, foundYellow = false;

  for (int i = 0; i < num_detections; i++) {
    float theta = detections[i].rotation_deg * M_PI / 180.0f;
    float d = detections[i].dist_cm;
    float x = d * sinf(theta);
    float y = d * cosf(theta);

    if (detections[i].label == 1) { // blue
      x1 = x; y1 = y; foundBlue = true;
    } else if (detections[i].label == 2) { // yellow
      x2 = x; y2 = y; foundYellow = true;
    }
  }

  if (foundBlue && foundYellow) {
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    heading = atan2f(dx, dy) * 180.0f / M_PI;
    heading *= -1.0f; // adjust direction
  }
}


void updateCM5() {
  if (Serial2.available()) {
    // read header
    const int num_detections_in = Serial2.read();

    if (num_detections_in == 0) {
      public_num_detections = 0;
      return;
    }

    const int stored_detections = (num_detections_in > MAX_DETECTIONS) ? MAX_DETECTIONS : num_detections_in;
    public_num_detections = stored_detections;

    // read label
    for (int i = 0; i < num_detections_in; i++) {
      while (Serial2.available() < 1) {}
      const uint8_t val = Serial2.read();
      if (i < stored_detections) {
        public_detections[i].label = val;
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
          public_detections[i].bbox[j] = halfToFloat(raw[j]);
        }
      }
    }

    // calculate bbox centers and then the rotation
    computeCenters(public_detections, stored_detections);
    computeRotations(public_detections, stored_detections);
    computeDistances(public_detections, stored_detections);
    //calibMirror(public_detections, stored_detections);

    // calculate robot heading based on goals
    computeHeadingFromPolar(public_detections, stored_detections);
  }
}
