//
// Created by julius on 05.01.2026.
//

#ifndef BOHLEBOTS_2026_SENSORS_H
#define BOHLEBOTS_2026_SENSORS_H

#include <util/Vector2.hpp>
#include <memory>
#include <comms/CM5.h>
#include <Arduino.h>

class Sensors {
public:
  explicit Sensors(const std::shared_ptr<CM5> &cm5);

  void update();

  void setEna(bool state);
  void allLEDsOff();
  void haltLEDs();

  [[nodiscard]] bool getButtonState(int device, int nr) const;
  void setLED(int device, int nr, int color);

  [[nodiscard]] int16_t getLineRot() const { return line_rot; }
  [[nodiscard]] int16_t getProgress() const { return progress; }
  [[nodiscard]] bool getLineSeen() const { return progress != -1 && line_rot != -1; }
  [[nodiscard]] Vector2 getPosition() const { return position; }
  [[nodiscard]] bool getEna() const { return ena; }
  [[nodiscard]] static bool getHasBall() { return analogRead(39) > 4000;}
  // static bool getHasBall() { return false; }

  enum COLOR { // for the LED colors
    OFF = 0,
    GREEN = 1,
    RED = 2,
    YELLOW = 3,
    BLUE = 4,
    CYAN = 5,
    MAGENTA = 6,
    WHITE = 7
  };

private:
  std::array<bool, 8> portEna = { true, false, false, false, false, false, false, false };

  std::array<int, 8> buttonLedID = { 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 };
  std::array<bool, 8> button1Array = { false, false, false, false, false, false, false, false };
  std::array<bool, 8> button2Array = { false, false, false, false, false, false, false, false };

  std::array<int, 8> led1Array = { 0, 0, 0, 0, 0, 0, 0, 0 };
  std::array<int, 8> led2Array = { 0, 0, 0, 0, 0, 0, 0, 0 };

  bool ena = false;
  bool lastButtonState = false;

  bool targetButtonState = false;

  int16_t line_rot = -1;
  int16_t progress = -1;

  std::shared_ptr<CM5> _cm5;

  Vector2 position;

  void updateLineSensor();

  void updateUS();

  void updateRunning();

  void updateButtons();

  void updateTargetButton();

  static void localToWorld(float lx, float ly, float heading_deg, float &gx, float &gy);
};


#endif //BOHLEBOTS_2026_SENSORS_H

