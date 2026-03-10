//
// GameStateStore.cpp – EEPROM persistence for role + target goal.
//

#include "util/GameStateStore.h"
#include <EEPROM.h>
#include <Arduino.h>

namespace GameStateStore {

    void save(const uint8_t role, const uint8_t targetGoal) {
        EEPROM.begin(8);
        EEPROM.write(ADDR_MAGIC,  MAGIC);
        EEPROM.write(ADDR_ROLE,   role);
        EEPROM.write(ADDR_TARGET, targetGoal);
        EEPROM.commit();
        EEPROM.end();
        Serial.println("[EEPROM] State saved: role=" + String(role) + " target=" + String(targetGoal));
    }

    SavedState load() {
        EEPROM.begin(8);
        const uint8_t magic  = EEPROM.read(ADDR_MAGIC);
        const uint8_t role   = EEPROM.read(ADDR_ROLE);
        const uint8_t target = EEPROM.read(ADDR_TARGET);
        EEPROM.end();

        if (magic != MAGIC) {
            Serial.println("[EEPROM] No valid saved state.");
            return {false, 0, 0};
        }

        Serial.println("[EEPROM] Restored state: role=" + String(role) + " target=" + String(target));
        return {true, role, target};
    }

    void clear() {
        EEPROM.begin(8);
        EEPROM.write(ADDR_MAGIC, 0x00);
        EEPROM.commit();
        EEPROM.end();
        Serial.println("[EEPROM] State cleared.");
    }

}