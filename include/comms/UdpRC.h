#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include <elapsedMillis.h>

struct __attribute__((packed)) RCCommand {
    int32_t moveX;
    int32_t moveY;
    int32_t rotation;
    uint8_t kick;
    int32_t dribblerSpeed; // 0 to 100
};

class UdpRC {
public:
    static UdpRC& getInstance();

    void init();
    void update();

    bool hasRecentCommand() const;
    RCCommand getCommand() const;

private:
    UdpRC() = default;

    WiFiUDP _udp;
    RCCommand _lastCmd = {0, 0, 0, false, 0};
    elapsedMillis _lastCmdTime;
    bool _initialized = false;
};
