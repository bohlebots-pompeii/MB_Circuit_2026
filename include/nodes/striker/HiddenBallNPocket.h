//
// Created by julius on 17.03.2026.
//

#pragma once

#include <bt/BehaviorNode.h>
#include <memory>

class MotionController;

class HiddenBallNPocket final : public BT::BehaviorNode {
public:
  explicit HiddenBallNPocket(std::shared_ptr<MotionController> motion);
  BT::Status tick(const WorldState& ws) override;

private:
  std::shared_ptr<MotionController> _motion;

  static bool checkBallInPocket(const WorldState& ws);
};
