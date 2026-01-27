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

extern bool isHoming;

class Bot
{
public:
    Bot();

    void update();

    static void overrideControl();

    // Set which goal to attack (1=blue, 2=yellow)
    void setTargetGoal(const uint8_t goalLabel) const { _cm5->setTargetGoal(goalLabel); }

private:
    std::shared_ptr<CM5> _cm5;
    std::shared_ptr<Sensors> _sensors;
    std::shared_ptr<Positioning> _positioning;

    // Helper functions
    [[nodiscard]] int getRotationControl(float input) ;
    [[nodiscard]] Vector2 getAwayFromLineVec(int speed);
    [[nodiscard]] Vector2 getMoveToCenterVec(int speed) const;
    [[nodiscard]] Vector2 getBallAlignedVec(int speed) const;
    [[nodiscard]] Vector2 getBallApproachVec(int speed) const;
    [[nodiscard]] Vector2 getBallPursuitVec() const;
    void updateYMotion(double y) const;
    void updateXMotion(double x) const;
    void updatePositionYAvg(double y);
    [[nodiscard]] bool checkBallOnLine() const;
    [[nodiscard]] Vector2 dribbleBackwards(int speed) const;
};
#endif //BOT_2026_BOT_H