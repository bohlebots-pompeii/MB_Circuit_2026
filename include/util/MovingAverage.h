#pragma once

#include <array>
#include <cstddef>

template <typename T, std::size_t N>
class MovingAverage {
public:
  MovingAverage();

  void addValue(T value);
  T getAverage() const;
  void reset();
  [[nodiscard]] bool isFull() const;
  [[nodiscard]] std::size_t getCount() const;

private:
  std::array<T, N> _values{};
  std::size_t _index{0};
  std::size_t _count{0};
  T _sum{0};
};

#include "util/MovingAverage.tpp"
