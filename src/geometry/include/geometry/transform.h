#pragma once

#include <optional>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include "algebra.h"

/* 变量说明
* @运动旋量(twist): [v,w];
* @四元数(quaternion): [w,x,y,z]; .coefff->[x,y,z,w];
* @李代数(lieAlgebra): so/se
* @李群(lieGroup): SO/SE
*/

/* 
* @brief : twist -> se(3)
* @param : vec - 旋转向量
* @return: vec 的反对称矩阵
*/
Eigen::Matrix4d liese3(Eigen::Vector<double,6> twist);

/* 
* @brief : vec -> SO(3)
* @param : vec - 旋转向量，已经包含旋转角度
* @return: vec 的反对称矩阵
*/
Eigen::Matrix3d lieso3(Eigen::Vector3d vec);

/* 
* @brief : twist * theta -> SE(3)
* @param : twist - 运动旋量，已经包含旋转角度
* @return: vec 的反对称矩阵
*/
Eigen::Isometry3d lieSE3(Eigen::Vector<double,6> twist);

std::optional<Eigen::Vector3d> so3toAxis(Eigen::Matrix3d skewSym);

/* 
* @brief : SE(3) -> twist
* @param : tran - 齐次转换矩阵
* @return: 齐次转换矩阵对应的运动旋量
*/
std::optional<Eigen::Vector<double,6>> SE3toTwist(Eigen::Isometry3d tran);


/* 
* @brief : 找到运动旋量上任意一点
* @param : twist - 运动旋量
* @return: 螺旋轴上任意一点 (p = p0 + wd)
*/
Eigen::Vector3d get_point_on_twist(Eigen::Vector<double,6> twist);

/* 
* @brief : Paden-Kahan subproblems
* @param : tran - 齐次转换矩阵
* @return: 齐次转换矩阵对应的运动旋量
*/
// Rotation about a single axis: [twist]p = q
std::optional<double> pk_subproblem_1(Eigen::Vector<double,6> twist, Eigen::Vector3d p, Eigen::Vector3d q);
// Rotation about two subsequent axes with intersecting point: [twist1][twist2]p = q
std::optional<std::vector<std::vector<double>>> pk_subproblem_2(Eigen::Vector<double,6> twist1, Eigen::Vector<double,6> twist2, Eigen::Vector3d p, Eigen::Vector3d q);
// Rotation to a given distance: |[twist]p - q| = d
std::optional<std::vector<double>> pk_subproblem_3(Eigen::Vector<double,6> twist, Eigen::Vector3d p, Eigen::Vector3d q, double distance);

