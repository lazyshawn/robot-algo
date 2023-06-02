#pragma once

#include <eigen3/Eigen/Dense>

/* 
* @brief : 点到平面的有向距离
* @param : target: 目标点
* @param : norm: 平面的外法向量(需先单位化)
* @param : viaPoint: 平面任意一点
* @return: 有向距离
*/
double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint);

/* 
* @brief : 主成分分析
* @param : points - n 个数据点坐标
* @param : axis - 主轴方向
* @param : com - 质心坐标
* @return: 
*/
Eigen::Vector3d principal_component_analysis(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& axis, Eigen::Vector3d& com);

