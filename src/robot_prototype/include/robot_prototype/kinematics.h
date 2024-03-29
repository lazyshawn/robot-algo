/**
* @file   kinematics.h
* @brief  机械臂正逆运动学算法.
*
* 正运动学, 子问题, 解析逆解, 数值逆解
*
* @see kinematics.cpp
*/
#pragma once

#include "geometry/transform.h"

/**
* @brief  顺序旋转的齐次转换矩阵
* @param  [in] jointAxis 关节螺旋轴
* @param  [in] theta     关节角 (rad)
* @param  [in] begIdx    起始螺旋轴的索引
* @param  [in] endIdx    终止螺旋轴的索引
* @return 齐次转换矩阵
*/
Eigen::Isometry3d serial_transfrom(const std::vector<Eigen::Vector<double, 6>>& jointAxis, const std::vector<double>& theta, size_t begIdx = 0, size_t endIdx = -1);

/**
* @brief  求解正运动学
* @param  [in] jointAxis 关节螺旋轴
* @param  [in] theta     关节角 (rad)
* @param  [in] M0        末端参考点的零位位姿
* @return 给定关节角状态下末端执行器的位姿
*/
Eigen::Isometry3d forward_kinematics(const std::vector<Eigen::Vector<double,6>>& jointAxis, const std::vector<double>& theta, Eigen::Isometry3d M0);

/**
* @brief  肘形机械臂逆运动学
*         第一组关节角的角度最小 - Inverse kinematics of elbow manipulator
* @param  [in] jointAxis 关节螺旋轴
* @param  [in] theta     关节角 (rad)
* @param  [in] M0        末端参考点的零位位姿
* @return 所有解的集合
*/
std::optional<std::vector<std::vector<double>>>
inverse_kinematics_elbow(const std::vector<Eigen::Vector<double, 6>>& jointAxis, Eigen::Isometry3d pose, Eigen::Isometry3d M0);

/**
* @brief  将关节角等价地转换到关节角限位区间内
* @param  [in] joint    关节角 (rad)
* @param  [in] interval 关节角限位区间
* @return 成功转换的标志位
*/
bool wrap_joint(std::vector<double>& joint, const std::vector<std::vector<double>>& interval);

/**
* @brief  获取关节限位空间内等价的关节状态
* @param  [in] joint    关节角 (rad)
* @param  [in] interval 关节角限位区间
*/
std::vector<std::vector<double>> get_equivalent_joint_state(const std::vector<double>& jointState, const std::vector<std::vector<double>>& interval);

/**
* @brief  查找到目标关节状态最近的一组关节状态
* @param  [in] jointStack 关节状态向量
* @param  [in] goal 目标关节角
*/
std::vector<double> get_nearest_joint_state(const std::vector<std::vector<double>>& jointStack, const std::vector<double>& goal = std::vector<double>());

/**
* @brief  检查关节状态是否在限位区间内
* @param  [in] joint    待检测的关节状态
* @param  [in] interval 关节限位区间
*/
bool satisfy_joint_limit(const std::vector<double>& joint, const std::vector<std::vector<double>>& interval);
