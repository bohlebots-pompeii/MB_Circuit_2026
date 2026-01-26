//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H

#include "comms/CM5.h"
#include "Sensors.h"
#include <memory>
#include <Positioning.h>
#include <Vector2.hpp>
#include <util/WebDebugger.h>

extern bool isHoming;

class Bot
{
public:
    Bot();

    void update();

    static void overrideControl();

    static void initDebugger(const char* ssid, const char* password);
    static void initDebuggerAP(const char* ssid, const char* password = nullptr);
private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<Positioning> _positioning;

    // Helper functions
    [[nodiscard]] int getRotationControl(float input) ;
    [[nodiscard]] Vector2 getAwayFromLineVec();
    [[nodiscard]] Vector2 getMoveToCenterVec(int speed) const;
    [[nodiscard]] Vector2 getBallAlignedVec(int speed) const;
    [[nodiscard]] Vector2 getBallApproachVec(int speed) const;
    [[nodiscard]] Vector2 getBallPursuitVec(int speed) const;
    void updateYMotion(double y) const;
    void updateXMotion(double x) const;
    void sendDebugData(const Vector2& target) const;
};
#endif //BOT_2026_BOT_H