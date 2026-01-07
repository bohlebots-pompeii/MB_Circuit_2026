//
// Created by julius on 06.01.2026.
//

#include "Positioning.h"
#include <memory>
#include <Arduino.h>

#include "Vector2.hpp"

Positioning::Positioning(const std::shared_ptr<CM5> &cm5) {
  _cm5 = cm5;
}

void Positioning::update() {
  updateMiddlePointVector();
}

void Positioning::updateMiddlePointVector() {
  const int x = _cm5->getGlobalX();
  const int y = _cm5->getGlobalY();

  // Vector from current pos to origin (0,0)
  _middlePointVector = Vector2(-x, -y);
}


