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
    void getSensorData();

    void pushData(bool enable, bool kick, int vx, int vy, uint8_t rotation, uint8_t dribbler);
};
#endif //BOT_2026_BOT_H