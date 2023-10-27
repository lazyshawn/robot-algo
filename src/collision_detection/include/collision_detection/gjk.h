/**
* @file   gjk.h
* @brief  GJK（Gilbert–Johnson–Keerthi）算法.
*
* 三维空间中的 GJK 检测
*
* @note 二维点集请转换为三维点集后使用.
*/
#pragma  once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cfloat>

/**
* @brief  计算凸集在给定方向上最远的点
* @param  [out] convexSet: 凸集
* @param  [in] dir: 搜索方向
* @return 最远的点(pnt)
* @par Sample
* @code
*   int x = 0;
* @endcode
*/
Eigen::Vector3d farthest_point_on_direction(std::vector<Eigen::Vector3d> convexSet, Eigen::Vector3d dir);

/**
* @brief  gjk 中的 support函数，搜索闵可夫斯基差在给定方向上的最远点
* @param  setA, setB: 凸集； dir: 搜索方向
* @return 闵可夫斯基差在给定方向上的最远点(pnt)
*/
Eigen::Vector3d gjk_support(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB, Eigen::Vector3d dir);

/**
* @brief  gjk 碰撞检测算法中更新单纯形和搜索方向
* @param  simplexList: 的单纯形；support: 新的 Support 点
* @return 更新单纯形后的搜索方向(dir)
*/
Eigen::Vector3d gjk_collision_update(std::vector<Eigen::Vector3d> &simplexList, Eigen::Vector3d support);

/**
* @brief  gjk 碰撞检测算法
* @param  setA, setB: 待检测的两个凸集
* @return 是否发生碰撞
* @ref    https://dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
*/
bool gjk_collision_detection(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB);

/**
* @brief  gjk 计算未碰撞的凸几何体之间的最小距离
* @param  setA, setB: 待检测的两个凸集
* @return 最小距离
*/
double gjk_closest_distance(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB);

/**
* @brief  gjk 计算最小距离算法中更新单纯形和搜索方向
* @param  simplexList: 单纯形；
* @param  support: 新的 Support 点
* @return 更新后的单纯形上距离原点最近的点
* @see    https://dyn4j.org/2010/04/gjk-distance-closest-points/
*/
Eigen::Vector3d gjk_closest_distance_update(std::vector<Eigen::Vector3d> &simplexList, Eigen::Vector3d support);

/**
* @brief  线段上距离原点最近的点
* @param  simplexList: 线段顶点
* @return 线段上距离原点最近的点(pnt)
*/
Eigen::Vector3d closest_point_on_line(std::vector<Eigen::Vector3d> simplexList);

/**
* @brief  三角形上距离原点最近的点
* @param  simplexList: 线段顶点
* @return 三角形上距离原点最近的点(pnt)
*/
Eigen::Vector3d closest_point_on_face(std::vector<Eigen::Vector3d> simplexList);

