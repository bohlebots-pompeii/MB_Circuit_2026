#include <memory>
#include "bot.h"
#include <util/log.h>
#include <config.h>
#include <Arduino.h>
#include <comms/esp-now.h>

Bot bot;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N2, 16, 17);
  bot.init();
  initEspNow();

  Log::header();
  Log::info(" Setup Complete; Switching to Loop");
}

void loop() {
  if (!overrideActive) {
    bot.update();
  }
  else {
    bot.overrideControl();
  }
  delay(10);
}