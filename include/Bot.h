//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H
#include <Arduino.h>

#include "comms/CM5.h"
#include "Sensors.h"

extern bool isHoming;

class Bot
{
public:
    Bot();

    void update();

    void overrideControl();

    void home();
private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
};
#endif //BOT_2026_BOT_H