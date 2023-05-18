#pragma once

#include <eigen3/Eigen/Dense>
#include <cfloat>

/* 
* @brief : 点到平面的有向距离
* @param : target: 目标点
* @param : norm: 平面的外法向量(需先单位化)
* @param : viaPoint: 平面任意一点
* @return: 有向距离
*/
double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint);

