#pragma once

#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

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
Eigen::Matrix<double,4,4> liese3(Eigen::Vector<double,6> twist);

/* 
* @brief : vec -> SO(3)
* @param : vec - 旋转向量
* @return: vec 的反对称矩阵
*/
Eigen::Matrix3d lieSO3(Eigen::Vector3d vec);

/* 
* @brief : twist * theta -> SE(3)
* @param : twist - 运动旋量，已经包含旋转角度
* @return: vec 的反对称矩阵
*/
Eigen::Isometry3d lieSE3(Eigen::Vector<double,6> twist);

