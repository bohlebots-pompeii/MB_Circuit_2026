//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H
#include <Arduino.h>

class Bot
{
public:
    void init();

    void update();
private:
    struct __attribute__((packed)) MotorCmd {
        uint8_t flags;   // bit0 = ena, bit1 = kick
        int8_t  vx;      // -100 .. 100
        int8_t  vy;      // -100 .. 100
        int8_t  rot;     // -100 .. 100
        int8_t  drib;    // -100 .. 100
    };

    float _x_pos = 0;
    float _y_pos = 0;

    int _head_calib = 0;
    int _head = 0;

    int _ena = 0;

    void getSensorData();

    int readCompass();

    void set_heading();

    void localToWorld(float lx, float ly, float heading_deg, float &gx, float &gy);

    void readButton();

    void pushData(bool enable, bool kick, int vx, int vy, int rot, int dribbler);
};
#endif //BOT_2026_BOT_H