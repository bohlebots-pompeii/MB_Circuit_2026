//
// Created by julius on 02.04.2026.
//

#pragma once

#include <memory>
#include <bt/BehaviorNode.h>

#include "util/Vector2.hpp"

class MotionController;

class PassBetween final : public BT::BehaviorNode {
public:
  explicit PassBetween(std::shared_ptr<MotionController> motion);
  BT::Status tick(const WorldState& ws) override;

private:
  std::shared_ptr<MotionController> _motion;
  static double calculateShotAngle(const WorldState& ws, const Vector2& targetPos);
};
