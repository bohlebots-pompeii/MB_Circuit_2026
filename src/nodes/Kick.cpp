#include <nodes/Kick.h>
#include <WorldState.h>
#include <MotionController.h>
#include <motor_mb.h>
#include <cmath>
#include <config/config.h>

Kick::Kick(std::shared_ptr<MotionController> motion)
    : BehaviorNode("Kick"), _motion(std::move(motion)) {
    _hasBallTimer = 0;
    _sequenceTimer = 0;
}

BT::Status Kick::tick(const WorldState& ws) {
    if (!ws.hasBall) {
        _hasBallTimer = 0;
        _state = KickState::IDLE;
        return BT::Status::FAILURE; // do nothing
    }

    if (!(std::abs(ws.targetGoalRot) < 15.0f)) {
        _hasBallTimer = 0;
        _state = KickState::IDLE;
        return BT::Status::FAILURE; // do nothing
    }

    if (_hasBallTimer < GeneralConfig::HasBallValidTime) {
        _state = KickState::IDLE;
        return BT::Status::FAILURE; // do nothing
    }

    if (_state == KickState::IDLE) {
        _state = KickState::PREPARING;
        _sequenceTimer = 0;
    }

    if (_state == KickState::PREPARING) {
        setDribbler(0);
        setKick(false);
        if (_sequenceTimer > 200) {
            _state = KickState::KICKING;
            _sequenceTimer = 0;
        }
        return BT::Status::RUNNING;
    }

    if (_state == KickState::KICKING) {
        setDribbler(0);
        setKick(true);
        if (_sequenceTimer > 100) { // Kick pulse 100ms
             _state = KickState::RESET;
             _sequenceTimer = 0;
        }
        return BT::Status::RUNNING;
    }

    if (_state == KickState::RESET) {
        _hasBallTimer = 0;
        _state = KickState::IDLE;
        return BT::Status::SUCCESS;
    }

    return BT::Status::FAILURE;
}
