#include <memory>
#include "Bot.h"
#include <util/log.h>
#include <Arduino.h>
#include <chrono>
#include <comms/esp-now.h>

std::shared_ptr<Bot> bot;

void setup() {
  bot = std::make_shared<Bot>();

  //bot->initDebugger("RoboAG", "------");

  //initEspNow();

  Log::header();
  Log::info(" Setup Complete; Switching to Loop");
}

void loop() {
  if (!overrideActive) {
    bot->update();
  }
  else {
    bot->overrideControl();
  }
  delay(1);
}