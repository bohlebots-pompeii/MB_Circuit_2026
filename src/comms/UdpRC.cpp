#include "comms/UdpRC.h"

UdpRC& UdpRC::getInstance() {
    static UdpRC instance;
    return instance;
}

void UdpRC::init() {
    // Enable both Station (for ESP-NOW) and Access Point
    WiFi.begin("BohleBots-Backrooms", "soccer2018");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    Serial.println(WiFi.localIP().toString().c_str());
    // Start UDP server on port 1337
    _udp.begin(1337);

    _initialized = true;
    Serial.println("[UPDRc] Running.");
}

void UdpRC::update() {
    if (!_initialized) return;

    int packetSize = _udp.parsePacket();
    if (packetSize) {
        // Prepare a buffer for the entire struct
        uint8_t buffer[sizeof(RCCommand)];
        int len = _udp.read(buffer, sizeof(RCCommand));

        if (len == sizeof(RCCommand)) {
            memcpy(&_lastCmd, buffer, sizeof(RCCommand));
            _lastCmdTime = 0; // reset timer
        }
    }
}

bool UdpRC::hasRecentCommand() const {
    // If we received a command within the last 500ms, considered active
    return _initialized && _lastCmdTime < 500;
}

RCCommand UdpRC::getCommand() const {
    return _lastCmd;
}

