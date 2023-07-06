// user_interface
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
double Timer::ms_since_starting() {
  using namespace std::chrono;
  time_point<system_clock> cur = system_clock::now();
  return duration_cast<duration<double,std::milli>>(cur - starting).count();
}

// 从上次计时到当前的毫秒数
double Timer::ms_since_last() {
  using namespace std::chrono;
  time_point<system_clock> cur = system_clock::now();
  return duration_cast<duration<double,std::milli>>(cur - *(timestamp.end()-1)).count();
}
