#pragma once
#include <bt/BehaviorNode.h>
#include <memory>
#include <elapsedMillis.h>

class MotionController;

class Kick final : public BT::BehaviorNode {
public:
    explicit Kick(std::shared_ptr<MotionController> motion);
    BT::Status tick(const WorldState& ws) override;

private:
    std::shared_ptr<MotionController> _motion;

    elapsedMillis _hasBallTimer; // Renaming for clarity if possible, but let's stick to existing or add new
    elapsedMillis _sequenceTimer;

    enum class KickState {
        IDLE,
        PREPARING,
        KICKING,
        RESET
    };
    KickState _state = KickState::IDLE;
};
