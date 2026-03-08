//
// Created by julius on 22.02.2026.
//

#ifndef BOHLEBOTS_2026_GAMESTATEHANDLER_H
#define BOHLEBOTS_2026_GAMESTATEHANDLER_H

#include <memory>
#include "Sensors.h"
#include "comms/CM5.h"

class GameStateHandler {
public:
    enum class State {
        TARGET_SELECT,  // right btn toggles blue/yellow
        LOCKED,         // blinks selected colour 2x, then advances
        ROLE_SELECT,    // left btn toggles striker/goalie
        RUNNING         // right btn stops -> back to ROLE_SELECT
    };

    enum class Role {
        STRIKER,
        GOALIE
    };

    GameStateHandler(std::shared_ptr<Sensors> sensors, std::shared_ptr<CM5> cm5);

    void update();

    [[nodiscard]] bool isRunning() const { return _state == State::RUNNING; }
    [[nodiscard]] Role getRole() const { return _role; }
    [[nodiscard]] State getState() const { return _state; }
    void setRole(const Role role) { _role = role; } // manually override role (in the switching)

private:
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<CM5>     _cm5;

    State _state = State::TARGET_SELECT;
    Role  _role  = Role::GOALIE;

    bool _targetIsYellow = false;   // false = BLUE, true = YELLOW

    uint8_t  _blinkCount    = 0;
    bool     _blinkLedOn    = false;
    uint32_t _blinkLastMs   = 0;
    static constexpr uint32_t BLINK_INTERVAL_MS = 250;
    static constexpr uint8_t  BLINK_TIMES       = 2;   // 2 full on/off cycles

    bool _lastLeft  = false;
    bool _lastRight = false;

    void handleTargetSelect();
    void handleLocked();
    void handleRoleSelect();
    void handleRunning();

    void applyRoleLED()   const;
    void applyTargetLED() const;
    void saveToEeprom()   const;

    bool _restoredFromEeprom = false;

    [[nodiscard]] int targetColor() const {
        return _targetIsYellow ? Sensors::YELLOW : Sensors::BLUE;
    }
};

#endif //BOHLEBOTS_2026_GAMESTATEHANDLER_H