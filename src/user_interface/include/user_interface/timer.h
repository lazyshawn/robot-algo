#pragma once

#include <chrono>
#include <vector>

// 时钟类型
enum class TimerClock {system_clock, steady_clock, high_resolution_clock};

class Timer {
  public:
  private:
  std::chrono::time_point<std::chrono::system_clock> starting;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> timestamp;

  public:
  void start();
  void record();
  void clear();
  double ms_since_starting();
  double ms_since_last();
};

