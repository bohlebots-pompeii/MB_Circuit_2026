//
// by Julius Gerhardus on 05.01.26
//
#include <comms/CM5.h>
#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <numbers>

Detection detections[6]; // memory because memory issues
Object objects[6] = {};
int num_detections = 0;
float heading = 0.0f;

static CalibPoint calib[] = {
  {  75.0f,  10.0f },
  { 100.0f,  20.0f },
  { 130.0f,  30.0f },
  { 148.0f,  40.0f },
  { 161.0f,  50.0f },
  { 169.0f, 60.0f },
  { 176.9f, 70.0f },
  { 183.7f, 80.0f },
  { 188.0f, 90.0f },
  { 191.6f,100.0f },
  { 194.5f,110.0f },
  { 195.8f,120.0f },
  { 197.2f,130.0f },
  { 198.7f,140.0f },
  { 200.8f,150.0f },
  { 202.7f,160.0f },
  { 203.9f,170.0f },
  { 206.4f,180.0f },
  { 208.35f,190.0f },
  { 211.4f,200.0f },
  { 213.8f,210.0f },
  { 215.8f,220.0f }
};

static int CALIB_N = std::size(calib);

float CM5::pixelToCm(const float pixel) {
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

void CM5::calibMirror(const Detection* det, const int num_det) {
  constexpr float cx = 320.0f; // screen center
  constexpr float cy = 320.0f;
  for (int i = 0; i < num_det; ++i) {
    if (det[i].label == 1 || det[i].label == 2) {
      const float dx = det[i].center[0] - cx;
      const float dy = cy - det[i].center[1];

      const float r = sqrtf(dx * dx + dy * dy);
      Serial.println(r);
    }
  }
}

float CM5::halfToFloat(const uint16_t h) {
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
    det[i].center[0] = (det[i].bbox[0] + det[i].bbox[2]) * 0.5f;
    det[i].center[1] = (det[i].bbox[1] + det[i].bbox[3]) * 0.5f;
  }
}

void CM5::computeRotations(const Detection* det, const int num_det) {
  constexpr float cx = 320.0f; // screen center
  constexpr float cy = 320.0f;
  for (int i = 0; i < num_det; ++i) {
    const float dx = det[i].center[1] - cx; // swap x and y to match ai output
    const float dy = cy - det[i].center[0];
    const float angle_rad = atan2f(dy, dx);
    objects[i].rotation_deg = angle_rad * 180.0f / std::numbers::pi;

    objects[i].label = det[i].label;
    if (det[i].label == 1) {
      blueRot = objects[i].rotation_deg;
    }
    else if (det[i].label == 2) {
      yellowRot = objects[i].rotation_deg;
    }
    else if (det[i].label == 3) {
      ballRot = objects[i].rotation_deg;
    }
  }
}

void CM5::computeDistances(const Detection* det, const int num_det) {
  constexpr float cx = 320.0f; // mirror center
  constexpr float cy = 320.0f;

  blueDist = 0;
  yellowDist = 0;
  ballDist = 0;

  for (int i = 0; i < num_det; ++i) {
    const float dx = det[i].center[1] - cx;
    const float dy = cy - det[i].center[0];

    const float r = sqrtf(dx * dx + dy * dy);
    const float dist = pixelToCm(r);

    objects[i].dist_cm = dist;

    if (objects[i].label == 1) {
      blueDist = dist;
      //Serial.print("blueDist: ");
      //Serial.print(blueDist);
    }
    else if (objects[i].label == 2) {
      yellowDist = dist;
      //Serial.print(" yellowDist: ");
      //Serial.println(yellowDist);
    }
    else if (objects[i].label == 3) {
      ballDist = dist;
    }
  }
}

double correction(const double x) {
  return 7.59502963e-06 * pow(x, 3) - 1.82918911e-03 * pow(x, 2) + 8.87600747e-01 * x - 4.79815488e-02;
}

void CM5::computeHeadingFromPolar(const Detection* det, const int num_det) {
  float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  bool foundBlue = false, foundYellow = false;

  for (int i = 0; i < num_det; i++) {
    const float theta = objects[i].rotation_deg * (std::numbers::pi / 180.0f);
    const float d = objects[i].dist_cm;
    const float x = d * cosf(theta);
    const float y = d * sinf(theta);

    if (det[i].label == 1) { // blue
      x1 = x; y1 = y; foundBlue = true;
    } else if (det[i].label == 2) { // yellow
      x2 = x; y2 = y; foundYellow = true;
    }
  }

  if (foundBlue && foundYellow) {
    const float dx = x2 - x1;
    const float dy = y2 - y1;
    const float h = -atan2f(dy, dx);
    heading = h * 180.0f / std::numbers::pi;

    const float mx = (x1 + x2) * 0.5f;
    const float my = (y1 + y2) * 0.5f;

    const float c = cosf(h);
    const float s = sinf(h);

    const float x = mx * c - my * s;
    const float y = mx * s + my * c;
    //x = static_cast<float>(correction(x));
    //y = static_cast<float>(correction(y));
    g_x = -x;
    g_y = -y;
  }
}

void CM5::update() {
  if (Serial2.available()) {
    // read header
    const int num_detections_in = Serial2.read();

    if (num_detections_in == 0) {
      num_detections = 0;
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

    computeCenters(detections, stored_detections);
    computeRotations(detections, stored_detections);
    computeDistances(detections, stored_detections);

    // calibMirror(detections, stored_detections);

    computeHeadingFromPolar(detections, stored_detections);
  }
}

float CM5::getHeading() const {
  return heading;
}