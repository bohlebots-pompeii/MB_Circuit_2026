//
// Created by julius on 05.01.2026.
//

#ifndef BOHLEBOTS_2026_SENSORS_H
#define BOHLEBOTS_2026_SENSORS_H

#include <Vector2.hpp>
#include <memory>
#include <comms/CM5.h>

class Sensors {
public:
  explicit Sensors(const std::shared_ptr<CM5> &cm5);

  void update();

  [[nodiscard]] int16_t getLineRot() const { return line_rot; }
  [[nodiscard]] int16_t getProgress() const { return progress; }
  [[nodiscard]] bool getLineSeen() const { return progress != -1 && line_rot != -1; }
  [[nodiscard]] Vector2 getPosition() const { return position; }
  [[nodiscard]] bool getEna() const { return ena; }

private:
  bool ena = false;
  bool lastButtonState = false;

  int16_t line_rot = -1;
  int16_t progress = -1;

  std::shared_ptr<CM5> _cm5;

  Vector2 position;

  void updateLineSensor();
  void updateUS();
  void updateButton();

  void localToWorld(float lx, float ly, float heading_deg, float &gx, float &gy);
};


#endif //BOHLEBOTS_2026_SENSORS_H

