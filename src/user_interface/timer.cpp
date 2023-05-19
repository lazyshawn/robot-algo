#include "user_interface/timer.h"

// 启动计时器
void Timer::start() {
  starting = std::chrono::system_clock::now();
  timestamp.push_back(std::chrono::system_clock::now());
}

// 记录当前时间
void Timer::record() {
  // 获取系统时间戳(ns)
  timestamp.push_back(std::chrono::system_clock::now());
}

// 清空计时器数据
void Timer::clear() {
  timestamp = std::vector<std::chrono::time_point<std::chrono::system_clock>>();
}


// 从开始到当前的毫秒数
std::chrono::duration<double, std::milli> Timer::time_since_starting() {
  std::chrono::time_point<std::chrono::system_clock> cur;
  cur = std::chrono::system_clock::now();
  return cur - starting;
}

// 从上次计时到当前的毫秒数
std::chrono::duration<double, std::milli> Timer::time_since_last() {
  std::chrono::time_point<std::chrono::system_clock> cur;
  cur = std::chrono::system_clock::now();
  return cur - *(timestamp.end()-1);
}

