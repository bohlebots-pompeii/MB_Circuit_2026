#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Robot A defends left (goalie default), Robot B attacks.
// ---------------------------------------------------------------------------
namespace EspNowConfig {
    constexpr uint8_t MAC_ROBOT_A[6] = {0x88, 0x13, 0xBF, 0xFA, 0x93, 0x68}; // Bot A
    constexpr uint8_t MAC_ROBOT_B[6] = {0x88, 0x13, 0xBF, 0xFA, 0x93, 0x10}; // TODO: fill in Bot B

    constexpr uint32_t TX_INTERVAL_MS = 10;
}

// Runtime bot ID – resolved automatically from own MAC in espNowInit().
// 0 = Robot-A, 1 = Robot-B, -1 = MAC not yet configured (uses broadcast)
extern int8_t espNowBotId;

// ---------------------------------------------------------------------------
// Data packet sent between robots  (~20 bytes packed)
// ---------------------------------------------------------------------------
struct __attribute__((packed)) EspNowPacket {
    float   globalX;        // robot global X position
    float   globalY;        // robot global Y position
    float   heading;        // robot heading in degrees
    float   ballRot;        // ball angle relative to robot
    float   ballDist;       // ball distance
    uint8_t flags;          // bit0=isRunning, bit1=isGoalie, bit2=seesLine, bit3=ballExists, bit 4=switchWanted
};

// ---------------------------------------------------------------------------
// Status flags helpers
// ---------------------------------------------------------------------------
inline void espNowSetFlag(uint8_t& flags, const uint8_t bit, const bool val) {
    if (val) flags |= (1u << bit); else flags &= ~(1u << bit);
}
inline bool espNowGetFlag(const uint8_t flags, const uint8_t bit) {
    return (flags >> bit) & 1u;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void espNowInit();

void espNowUpdate(const EspNowPacket& myData);

const EspNowPacket& espNowGetPeerData();

bool espNowPeerAlive();

bool espNowPeerKnown();

String espNowGetOwnMac();
