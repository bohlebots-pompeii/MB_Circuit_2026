//
// Created by julius on 11.11.2025.
//

#include <Bot.h>
#include <Arduino.h>
#include <Wire.h>
#include <memory>
#include <motor_mb.h>

Bot::Bot() {
    Wire.begin();
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N2, 16, 17);

    _cm5 = std::make_shared<CM5>();
    _sensors = std::make_shared<Sensors>(_cm5);
    _positioning = std::make_shared<Positioning>(_cm5);

    _striker = std::make_unique<Striker>(_cm5, _sensors, _positioning);
    _goalie = std::make_unique<Goalie>(_cm5, _sensors, _positioning);
}

void Bot::updateStriker() const {
    _striker->update();
}

void Bot::updateGoalie() const {
    _goalie->update();
}

void Bot::overrideControl() {
    pushData(false, false, 0, 0, 0, 0);
}