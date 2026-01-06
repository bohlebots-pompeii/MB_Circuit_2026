//
// Created by julius on 06.01.2026.
//

#include "Positioning.h"
#include <memory>
#include <Arduino.h>

Positioning::Positioning(const std::shared_ptr<Sensors> &sensors) {
  _sensors = sensors;
}

void Positioning::update() {
  updateMiddlePointVector();
}

void Positioning::updateMiddlePointVector() {
  const Vector2 pos = _sensors->getPosition();
  const float x = pos.getX();
  const float y = pos.getY();

  // Vector from current pos to origin (0,0)
  _middlePointVector = Vector2(-x, -y);
}


