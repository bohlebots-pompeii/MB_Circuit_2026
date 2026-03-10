#pragma once
//
// Persists role + target-goal to EEPROM so the bot can resume after a brief
// power loss without going through the full menu sequence again.
//
// Layout (3 bytes, starting at EEPROM address 0):
//   [0]  magic byte  – 0xBB if data is valid
//   [1]  role        – 0 = GOALIE, 1 = STRIKER
//   [2]  targetGoal  – 0 = BLUE,   1 = YELLOW
//

#include <Arduino.h>
#include <cstdint>

namespace GameStateStore {
    static constexpr uint8_t MAGIC       = 0xBB;
    static constexpr int     ADDR_MAGIC  = 0;
    static constexpr int     ADDR_ROLE   = 1;
    static constexpr int     ADDR_TARGET = 2;

    struct SavedState {
        bool valid;
        uint8_t role;        // 0 = GOALIE, 1 = STRIKER
        uint8_t targetGoal;  // 0 = BLUE,   1 = YELLOW
    };

    void       save(uint8_t role, uint8_t targetGoal);
    SavedState load();
    void       clear();
}