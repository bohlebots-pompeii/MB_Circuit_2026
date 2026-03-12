//
// Created by julius on 24.02.2026.
//
// reference: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html
//

#include "comms/esp-now.h"
#include <config/config_esp_now.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

int8_t espNowBotId = -1;

namespace {
    EspNowPacket s_peerData = {};
    uint32_t s_lastRxMs = 0;
    uint32_t s_lastTxMs = 0;
    bool s_peerAdded = false;
    bool s_initialized = false;

    bool isMacAllFF(const uint8_t* mac) {
        if (mac == nullptr) return true; // null = not configured
        for (int i = 0; i < 6; i++) if (mac[i] != 0xFF) return false;
        return true;
    }

    bool macEqual(const uint8_t* a, const uint8_t* b) {
        return memcmp(a, b, 6) == 0;
    }

    const uint8_t* getPeerMac() {
        if (espNowBotId == 0) return EspNowConfig::MAC_ROBOT_B;
        if (espNowBotId == 1) return EspNowConfig::MAC_ROBOT_A;
        return nullptr;
    }

    void IRAM_ATTR onDataRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
        if (len == sizeof(EspNowPacket)) {
            memcpy(&s_peerData, data, sizeof(EspNowPacket));
            s_lastRxMs = millis();
        }
    }

    void IRAM_ATTR onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t /*status*/) {
        // no callback
    }

    bool addPeer(const uint8_t* mac) {
        if (esp_now_is_peer_exist(mac)) return true;

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.encrypt = false;
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("[ESP-NOW] Failed to add peer");
            return false;
        }
        Serial.println("[ESP-NOW] Peer added");
        return true;
    }
}

void espNowInit() {
    WiFiClass::mode(WIFI_STA);
    WiFi.disconnect();

    // Change between long range and high throughput PHY modes as needed; See docs for details.
    // Set long-range / high-throughput PHY -> See Docs
    // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR); // use LR for max range

    // maximum throughput -> look at Docs
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    s_initialized = true;

    uint8_t ownMac[6];
    esp_wifi_get_mac(WIFI_IF_STA, ownMac);

    if (!isMacAllFF(EspNowConfig::MAC_ROBOT_A) && macEqual(ownMac, EspNowConfig::MAC_ROBOT_A)) {
        espNowBotId = 0;
    } else if (!isMacAllFF(EspNowConfig::MAC_ROBOT_B) && macEqual(ownMac, EspNowConfig::MAC_ROBOT_B)) {
        espNowBotId = 1;
    } else {
        espNowBotId = -1;
    }

    Serial.print("[ESP-NOW] Own MAC: ");
    Serial.println(espNowGetOwnMac());
    Serial.print("[ESP-NOW] Bot ID : ");
    if (espNowBotId >= 0) {
        Serial.println(espNowBotId);
    }

    const uint8_t broadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    addPeer(broadcast);

    if (const uint8_t* peerMac = getPeerMac(); peerMac != nullptr && !isMacAllFF(peerMac)) {
        s_peerAdded = addPeer(peerMac);
    } else {
        Serial.println("[ESP-NOW] Peer MAC not yet configured");
    }

    Serial.println("[ESP-NOW] Ready");
}

void espNowUpdate(const EspNowPacket& myData) {
    if (!s_initialized) return;

    const uint32_t now = millis();
    if (now - s_lastTxMs < EspNowConfig::TX_INTERVAL_MS) return;
    s_lastTxMs = now;

    // Prefer unicast to the known peer; fall back to broadcast
    if (const uint8_t* peerMac = getPeerMac(); peerMac == nullptr || isMacAllFF(peerMac)) {
        // Bot ID unknown or peer not configured – broadcast
        uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        esp_now_send(bcast, reinterpret_cast<const uint8_t*>(&myData), sizeof(myData));
    } else {
        if (!s_peerAdded) {
            s_peerAdded = addPeer(peerMac);
        }
        esp_now_send(peerMac, reinterpret_cast<const uint8_t*>(&myData), sizeof(myData));
    }
}

const EspNowPacket& espNowGetPeerData() {
    return s_peerData;
}

bool espNowPeerAlive() {
    return millis() - s_lastRxMs < 1000;
}

bool espNowPeerKnown() {
    const uint8_t* peerMac = getPeerMac();
    return peerMac != nullptr && !isMacAllFF(peerMac);
}

String espNowGetOwnMac() {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return {buf};
}
