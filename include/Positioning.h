//
// Created by julius on 06.01.2026.
//

#ifndef BOHLEBOTS_2026_POSITIONING_H
#define BOHLEBOTS_2026_POSITIONING_H

#include <memory>
#include <comms/CM5.h>


class Positioning {
public:
  explicit Positioning(const std::shared_ptr<CM5> &cm5);

  void update();

  [[nodiscard]] Vector2 getMiddlePointVector() const { return _middlePointVector; }
private:
  std::shared_ptr<CM5> _cm5;
  Vector2 _middlePointVector;

  void updateMiddlePointVector();
};


#endif //BOHLEBOTS_2026_POSITIONING_H