#include "comms/esp-now.h"
#include <esp_now.h>
#include <WiFi.h>
#include "bot.h"

bool isHoming = false;
bool overrideActive = false;

uint8_t senderMacAddress[] = {0x40, 0x22, 0xD8, 0x5F, 0xB4, 0x84};

struct_message receivedData;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, const int len) {
    Serial.println("onDataRecv");
    if (len == sizeof(struct_message)) {
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        if (receivedData.homing) {
            isHoming = true;
            Serial.println("homing...");
        } else {
            overrideActive = !overrideActive;
            Serial.print("overrideActive toggled to: ");
            Serial.println(overrideActive);
        }
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, senderMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}
