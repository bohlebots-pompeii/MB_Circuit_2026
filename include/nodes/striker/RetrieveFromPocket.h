//
// Created by julius on 17.03.2026.
//

#pragma once

#include <bt/BehaviorNode.h>
#include <memory>
#include <elapsedMillis.h>

class MotionController;

class RetrieveFromPocket final : public BT::BehaviorNode {
public:
  explicit RetrieveFromPocket(std::shared_ptr<MotionController> motion);
  BT::Status tick(const WorldState& ws) override;

private:
  std::shared_ptr<MotionController> _motion;

  static bool checkBallInPocket(const WorldState& ws);
};
