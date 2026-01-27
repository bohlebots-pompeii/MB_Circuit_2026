#pragma once
#include <util/MovingAverage.h>

#ifndef MOVINGAVERAGE_TPP
#define MOVINGAVERAGE_TPP

template<typename T, std::size_t N>
MovingAverage<T, N>::MovingAverage() {
  reset();
}

template<typename T, std::size_t N>
void MovingAverage<T, N>::addValue(const T value) {
  _sum -= _values[_index];
  _values[_index] = value;
  _sum += value;

  _index = (_index + 1) % N;

  if (_count < N) {
    _count++;
  }
}

template<typename T, std::size_t N>
T MovingAverage<T, N>::getAverage() const {
  if (_count == 0) {
    return T{0};
  }
  return _sum / static_cast<T>(_count);
}

template<typename T, std::size_t N>
void MovingAverage<T, N>::reset() {
  _values.fill(T{0});
  _index = 0;
  _count = 0;
  _sum = T{0};
}

template<typename T, std::size_t N>
bool MovingAverage<T, N>::isFull() const {
  return _count >= N;
}

template<typename T, std::size_t N>
std::size_t MovingAverage<T, N>::getCount() const {
  return _count;
}

#endif // MOVINGAVERAGE_TPP
