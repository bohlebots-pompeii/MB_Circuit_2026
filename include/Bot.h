//
// Created by julius on 11.11.2025.
//

#ifndef BOT_2026_BOT_H
#define BOT_2026_BOT_H

#include "comms/CM5.h"
#include "comms/esp-now.h"
#include "Sensors.h"
#include <memory>
#include <Positioning.h>
#include "Striker.h"
#include "Goalie.h"
#include "GameStateHandler.h"

class Bot {
public:
    Bot();

    void update() const;

private:
    std::shared_ptr<CM5>          _cm5;
    std::shared_ptr<Sensors>      _sensors;
    std::shared_ptr<Positioning>  _positioning;
    std::unique_ptr<Striker>      _striker;
    std::unique_ptr<Goalie>       _goalie;
    std::unique_ptr<GameStateHandler> _gameState;

    static void printPeerPacket();

    static bool getSwitchWantedFromPeer() {
        const auto& pkt = espNowGetPeerData();
        return espNowGetFlag(pkt.flags, 4);
    }

    void fillEspNowPacket(EspNowPacket& pkt) const;
};

#endif //BOT_2026_BOT_H

