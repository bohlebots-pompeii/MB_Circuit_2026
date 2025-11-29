#include <memory>
#include "bohlebots.h"
#include "../include/util/log.h"

std::unique_ptr<BohleBots> bot;

// hello webhook

void setup() {
    Log::header();
    Log::info("Setup complete");
}

void loop() {
    bot->update();
}