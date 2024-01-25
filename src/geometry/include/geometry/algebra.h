/**
* @file   algebra.h
* @brief  Basic algebra
*
* 常见代数运算
*/
#pragma once

#include <list>

#include "algebra.hpp"

/**
* @brief  点到平面的有向距离
* @param  target   目标点
* @param  norm     平面的外法线方向 (需先单位化)
* @param  viaPoint 平面任意一点
* @return 有向距离, (+) 面上方, (-) 面下方
*/
double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint);

/**
* @brief  沿给定方向排序
* @param  [out] points 点集
* @param  direction    排序方向
*/
void sort_along_direction(std::vector<Eigen::Vector3d>& points, Eigen::Vector3d direction);

/**
* @brief  用给定向量构造正交基底
*
* 第一列为输入的 vec
*
* @param  vec 三维空间中一个向量
* @return 单位正交矩阵
*/
Eigen::Matrix3d construct_unit_orthogonal_basis(Eigen::Vector3d vec);

/**
* @brief  判断浮点数是否接近零
* @param  value   待判断的浮点数
* @param  ellipse 允许误差
* @return True, False
*/
bool is_near_zero(double value, double ellipse = 1e-15);

/**
* @brief  最小二乘拟合空间圆弧
* @param  samples  采样点
* @param  [out] center 拟合的圆心坐标
* @param  relErr 采样点到圆心的距离与半径的平均误差(absErr)与半径的比值
* @return 拟合圆的半径，当 relErr 大于阈值时返回结果为负
* @see    https://blog.csdn.net/weixin_46581517/article/details/105178304
*/
double circle_fitting(const std::vector<Eigen::Vector3d>& samples, Eigen::Vector3d& center, double relErr = 1e-2);

/**
* @brief  Circumcenter of triangle
* @param  beg    coordinate of beginning point
* @param  mid    coordinate of middle point
* @param  end    coordinate of end point
* @return Circumcenter of given triangle, return {inf, inf, inf} when colinear
*/
Eigen::Vector3d triangular_circumcenter(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end);

