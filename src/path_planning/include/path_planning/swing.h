/**
* @file   swing.h
* @brief  headers for swing welding
*/
#pragma once

#include <functional>
#include <map>

#include "geometry/algebra.h"


/**
* @brief  摆动补偿类型
*/
enum class SwingType {
  SIN,      //!< sin
  CIRCLE,   //!< 圆形
  L,        //!< L 型
  LEMNI     //!< 8 型
};

/**
* @brief  基础轨迹(line/circle)的摆动插值
* @param  traj    轨迹点，直线两点，圆弧三点
* @param  type    摆动偏移的类型
* @return 轨迹点  vector<Eigen::vector6d>, 轨迹点表示方式为<pos, dir>
* @todo
* 1. 补充形参: 上方向, 振幅, 频率
* 2. 按时间插补
*/
std::vector<Eigen::Vector<double,6>> swing_interpolation(const std::vector<Eigen::Vector3d>& traj, SwingType type);

/**
* @brief  基础轨迹(line/circle)的摆动插值
* @param  traj          轨迹点，直线两点，圆弧三点
* @param  delT          插值周期时长            ms
* @param  upper         轨迹上方向
* @param  type          摆动偏移的类型
* @param  vel           焊接速度                mm/s
* @param  freq          摆动频率                Hz
* @param  leftHoldT     左停留时间              ms
* @param  rightHoldT    右停留时间              ms
* @param  amplitude     偏移方向振幅(左右)      mm
* @param  radius        运动方向振幅(前后)      mm
* @return 轨迹点        vector<Eigen::vector6d>, 轨迹点表示方式为<pos, dir>
*/
std::vector<Eigen::Vector<double, 6>>
swing_interpolation(const std::vector<Eigen::Vector3d> &traj, double delT,
                    Eigen::Vector3d upper, SwingType type, double vel,
                    double freq = 1.0, double leftHoldT = 0.0,
                    double rightHoldT = 0.0, double amplitude = 1.0,
                    double radius = 1.0);

/**
* @brief  sin 函数摆动的补偿向量插值
* @param  phase        当前点位所处的相位角
* @param  offsetDir    偏移方向，单位化
* @param  dir          进给方向，单位化
* @param  amplitude    偏移方向的摆动幅值
* @param  radius       进给方向摆动幅值
* @return 补偿向量     Eigen::Vector3d    offsetVec
*/
Eigen::Vector3d sin_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius);

/**
* @brief  圆形摆动的补偿向量插值
* @param  phase        当前点位所处的相位角
* @param  offsetDir    偏移方向，单位化
* @param  dir          进给方向，单位化
* @param  amplitude    偏移方向的摆动幅值
* @param  radius       进给方向摆动幅值
* @return 补偿向量     Eigen::Vector3d    offsetVec
*/
Eigen::Vector3d circle_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius);

/**
* @brief  双扭线(8字型, Lemniscate of Bernoulli)摆动的补偿向量插值
* @param  phase        当前点位所处的相位角
* @param  offsetDir    偏移方向，单位化
* @param  dir          进给方向，单位化
* @param  amplitude    偏移方向的摆动幅值
* @param  radius       进给方向摆动幅值
* @return 补偿向量     Eigen::Vector3d    offsetVec
*/
Eigen::Vector3d lemniscate_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius);

/**
* @brief  L 型摆动的补偿向量插值
* @param  phase        当前点位所处的相位角
* @param  offsetDir    偏移方向，单位化
* @param  dir          进给方向，单位化
* @param  amplitude    偏移方向的摆动幅值
* @param  radius       进给方向摆动幅值
* @return 补偿向量     Eigen::Vector3d    offsetVec
*/
Eigen::Vector3d L_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius);

