#include <memory>
#include "Bot.h"
#include <util/log.h>
#include <Arduino.h>
#include <comms/esp-now.h>

std::shared_ptr<Bot> bot;

void setup() {
    Serial.begin(115200);
    espNowInit();
    bot = std::make_shared<Bot>();
    Log::header();
    Log::info("Setup Complete!");
}

void loop() {
    bot->update();

    delay(1);
}