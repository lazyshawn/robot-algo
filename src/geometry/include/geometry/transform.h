#pragma once

#include <optional>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <map>

#include "algebra.h"
#ifndef M_PI
constexpr double M_PI = 3.1415926535897932384626433832795028841971693993751058209;
#endif

/* 变量说明
* @运动旋量(twist): [v,w];
* @四元数(quaternion): [w,x,y,z]; .coefff->[x,y,z,w];
* @李代数(lieAlgebra): so/se
* @李群(lieGroup): SO/SE
*/

/* 
* @brief : 欧拉角转旋转矩阵
* @param : endIdx - 终止螺旋轴的索引
* @return: 齐次转换矩阵
*/
Eigen::Matrix3d euler2SO3(std::vector<double> theta, std::vector<size_t> axisIdx = {2,1,0});

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
Eigen::Matrix3d lieSO3(Eigen::Vector3d vec);

/* 
* @brief : twist * theta -> SE(3)
* @param : twist - 运动旋量，已经包含旋转角度
* @return: vec 的反对称矩阵
*/
Eigen::Isometry3d lieSE3(Eigen::Vector<double,6> twist);

/* 
* @brief : 反对称矩阵转旋转轴
* @param : so3 - 反对称矩阵
* @return: 单位旋转轴
*/
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
* @brief : 求相交的螺旋轴的交点
* @param : twist - 若干相交的螺旋轴组成的向量
* @return: 螺旋轴的交点
* @ref   : https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
*/
Eigen::Vector3d get_twist_intersection(std::vector<Eigen::Vector<double,6>> twist);

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

/* 
* @brief : canonical subproblems
* @param : 
* @return: 
* @ref   : "Canonical subproblems for Robot Inverse Kinematics", Alexander J. Elias
* @Todos : 共线判断
*/
// min|h^T R(k,q) p - d|: Circle and plane, 最小化旋转后的投影
std::vector<double> canonical_subproblem_4(Eigen::Vector3d p, Eigen::Vector3d k, Eigen::Vector3d h, double d);
bool verify_canonical_subproblem_4(Eigen::Vector3d p, Eigen::Vector3d k, Eigen::Vector3d h, double d, double q);

// min|R(k,q) p1 - p2|: Circle and point, 最小化旋转后的距离
double canonical_subproblem_1(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2);
bool verify_canonical_subproblem_1(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double q);

// min|R(k1,q1) p1 - R(k2,q2)p2|: Two circles, 最小化两点旋转的差
std::vector<std::vector<double>> canonical_subproblem_2(Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d p2, Eigen::Vector3d k2);
bool verify_canonical_subproblem_2(Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d p2, Eigen::Vector3d k2, double q1, double q2);

// min| ||R(k,q)p1 - p2|| - d |: Circle and sphere, 最小化旋转后到球面的距离
std::vector<double> canonical_subproblem_3(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double d);
bool verify_canonical_subproblem_3(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double d, double q);

// h1^T R(k1,q1) p1 + h2^T R(k2,q2) p2 = d1
// h3^T R(k3,q3) p3 + h4^T R(k4,q4) p4 = d2
std::vector<std::vector<double>> canonical_subproblem_6(
    Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d h1,
    Eigen::Vector3d p2, Eigen::Vector3d k2, Eigen::Vector3d h2, double d1,
    Eigen::Vector3d p3, Eigen::Vector3d k3, Eigen::Vector3d h3,
    Eigen::Vector3d p4, Eigen::Vector3d k4, Eigen::Vector3d h4, double d2);

/* 
* @brief : 判断给定系数的二次型是否表示一个椭圆
* @param : k - [1,x,y,x2,xy,y] 的系数, y 的系数归一
* @return: true / false
*/
bool is_ellipse(const std::vector<double>& k);

/* 
* @brief : 查找两个椭圆的交点
* @param : k1 - 椭圆 1 的二次型系数, k2 - 椭圆 2 的二次型系数
* @return: 
*/
std::vector<std::vector<double>> find_intersection_of_ellipses(const std::vector<double>& a, const std::vector<double>& b);

/* 
* @brief : 一元二次方程求根公式
* @param : k - [1,x,x2] 的系数
* @return: <root, multiplicity>
*/
std::map<double, int> solve_quadratic_equation(const std::vector<double>& k);

/* 
* @brief : 一元三次方程求根公式
* @param : k - [1,x,x2,x3] 的系数
* @return: <root, multiplicity>
*/
std::map<double, int> solve_cubic_equation(const std::vector<double>& k);

/* 
* @brief : 一元四次方程求根公式
* @param : k - [1,x,x2,x3,x4] 的系数
* @return: <root, multiplicity>
* @ref   : https://en.wikipedia.org/wiki/Quartic_equation
*/
std::map<double, int> solve_quartic_equation(const std::vector<double>& k);

