//
// Created by julius on 19.12.2025.
//

#include "util/piezo.h"
#include <Arduino.h>
#include <esp32-hal-ledc.h>

Piezo::Piezo(const uint8_t pin, const uint8_t channel, const uint32_t freq, const uint8_t resolution)
    : _pin(pin), _channel(channel), _baseFreq(freq), _resolution(resolution) {}

void Piezo::begin() const {
    ledcAttach(_pin, 1000, 8);
    ledcWrite(_channel, 0); // sicherstellen, dass nichts spielt
}

uint32_t Piezo:: _maxDuty() const {
    return (1u << _resolution) - 1u;
}

void Piezo::playTone(const uint32_t frequency, const uint32_t duration_ms, uint32_t duty) const {
    if (duty > _maxDuty()) duty = _maxDuty();
    if (frequency == 0 || duty == 0) {
        stop();
        if (duration_ms) delay(duration_ms);
        return;
    }
    ledcWriteTone(_channel, frequency); // setzt die PWM-Frequenz
    ledcWrite(_channel, duty);          // setzt Duty
    if (duration_ms) {
        delay(duration_ms);
        stop();
    }
}

void Piezo::stop() const {
    ledcWriteTone(_channel, 0);     // stopt Tone
    ledcWrite(_channel, 0);         // setzt Duty auf 0
}

void Piezo::setDuty(uint32_t duty) const {
    if (duty > _maxDuty()) duty = _maxDuty();
    ledcWrite(_channel, duty);
}
