#include <memory>
#include "bot.h"
#include <util/log.h>
#include <config.h>
#include <Arduino.h>

Bot bot;

void setup() {
  Serial.begin(115200);
  bot.init();

  Log::header();
  Log::info(" Setup Complete; Switching to Loop");
}

void loop() {
  bot.update();

  delay(10);
}