/**
* @file   timer.h
* @brief  计时器操作
*/
#pragma once

#include <chrono>
#include <vector>

/**
 * @brief  A timer manager.
 *
 * 记录时间戳，计算运行时间
 */
class Timer {
public:
  /**
   *  chrono 使用的时钟类型
   */
  enum class TimerClock {
    system_clock,         //!< 系统时钟
    steady_clock,         //!< 固定时钟
    high_resolution_clock //!< 高精度时钟
  };

private:
  std::chrono::time_point<std::chrono::system_clock> starting;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> timestamp;

public:
  /**
  * @brief  开始计时，记录下第一个时间戳
  */
  void start();
  /**
  * @brief  记录一个时间戳
  */
  void record();
  /**
  * @brief  清除所有记录的时间戳
  */
  void clear();
  /**
  * @brief  从开始计时到当前时刻的毫秒数
  */
  double ms_since_starting();
  /**
  * @brief  从上一次计时到当前时刻的毫秒数
  */
  double ms_since_last();
};

