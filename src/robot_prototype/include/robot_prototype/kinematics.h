#pragma once

#include "geometry/transform.h"

/* 
* @brief : 求解正运动学
* @param : jointAxis - 关节螺旋轴
* @param : theta (rad) - 关节角
* @param : M0 - 末端参考点的零位位姿
* @return: 给定关节角状态下末端执行器的位姿
*/
Eigen::Isometry3d forward_kinematics(const std::vector<Eigen::Vector<double,6>>& jointAxis, const std::vector<double>& theta, Eigen::Isometry3d M0);

/* 
* @brief : 肘形机械臂逆运动学 - Inverse kinematics of elbow manipulator
* @param : jointAxis - 关节螺旋轴
* @param : theta (rad) - 关节角
* @param : M0 - 末端参考点的零位位姿
* @return: 所有解的集合
*/
std::optional<std::vector<std::vector<double>>>
inverse_kinematics_elbow(const std::vector<Eigen::Vector<double, 6>>& jointAxis, Eigen::Isometry3d pose, Eigen::Isometry3d M0);

/* 
* @brief : 将关节角等价地转换到关节角限位区间内
* @param : joint (rad) - 关节角
* @param : interval - 关节角限位区间
* @return: 成功转换的标志位
*/
bool wrap_joint(std::vector<double>& joint, const std::vector<std::vector<double>>& interval);
