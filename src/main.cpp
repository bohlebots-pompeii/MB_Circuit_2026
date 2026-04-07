#include <memory>
#include <Bot.h>
#include <Arduino.h>
#include <comms/esp-now.h>

std::shared_ptr<Bot> bot;

void setup() {
  Serial.begin(115200);
  espNowInit();
  bot = std::make_shared<Bot>();
}

void loop() {
  bot->tick();

  delay(1);
}