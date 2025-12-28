//
// Created by julius on 19.12.2025.
//

#ifndef BOHLEBOTS_2026_PIEZO_H
#define BOHLEBOTS_2026_PIEZO_H

#pragma once
#include <Arduino.h>

class Piezo {
public:
    explicit Piezo(uint8_t pin, uint8_t channel = 0, uint32_t freq = 2000, uint8_t resolution = 8);

    void begin () const;

    void playTone(uint32_t frequency, uint32_t duration_ms, uint32_t duty) const;

    void stop() const;

    void setDuty(uint32_t duty) const;

private:
    uint8_t _pin;
    uint8_t _channel;
    uint32_t _baseFreq;
    uint8_t _resolution;
    uint32_t _maxDuty() const;
};


#endif //BOHLEBOTS_2026_PIEZO_H