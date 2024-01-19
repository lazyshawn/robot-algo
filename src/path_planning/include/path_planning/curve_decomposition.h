/**
* @file   curve_decomposition.h
* @brief  Decomposition of a curve into arcs and line segments based on
*         dominant point detection
*/
#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

/**
* @brief  Discrete trajectory with only arcs and straight line segments
*
* curve decomposition, arc approximating, pose calculation, and etc.
*/
class DiscreteTrajectory {
public:
  //! 轨迹上有序的离散点坐标
  std::vector<Eigen::Vector3d> pntList;
  //! 离散轨迹端点上的位姿
  std::vector<Eigen::Isometry3d> pntPose;
  //! 离散轨迹上圆弧点处的姿态，比位置点数量少一
  std::vector<Eigen::Isometry3d> midPose;
  //! 圆弧半径
  std::vector<double> radii;
  //! 离散轨迹的分割点索引
  std::vector<size_t> sepIdx;
  //! 圆弧点的索引
  std::vector<size_t> midIdx;

public:
  DiscreteTrajectory();
  DiscreteTrajectory(const std::vector<Eigen::Vector3d>& points);
  ~DiscreteTrajectory(){};

  /**
  * @brief  加载数据点
  * @param  point 数据点
  */
  void load_trajectory(const std::vector<Eigen::Vector3d>& points);
  /**
  * @brief  将轨迹分割，保存分割点的索引
  * @param  linearErr    直线近似误差
  * @param  curveErr     曲率分割误差
  */
  void perform_trajectory_decomposition(double linearErr = 8e-2, double curveErr = 8e-2);
  /**
  * @brief  计算离散点和圆弧点上的位姿
  * @param  upper    轨迹上表面方向
  */
  void calc_trajectory_pose(Eigen::Vector3d upper = {0,0,1});
  /**
  * @brief  清除数据
  */
  void clear();

};

/**
* @brief  Circumcenter of triangle
* @param  beg    coordinate of beginning point
* @param  mid    coordinate of middle point
* @param  end    coordinate of end point
* @return Circumcenter of given triangle, return {inf, inf, inf} when colinear
*/
Eigen::Vector3d triangular_circumcenter(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end);

/**
* @brief  Split a curve into maximal blurred segments (MBS)
* @param  pntList    sequential points on curve
* @param  epsilon    maximun distance from a point to its MBS
* @return index of dominant points which is the seperation for adjacent MBS
*/
std::vector<size_t> maximal_blurred_segment_split(const std::vector<Eigen::Vector3d>& pntList, double epsilon);

/**
* @brief  Get mid points curve (MPC) for set of line segments from its tangent space
* @param  pntList    sequential points on curve
* @param  dpIdx index of dominant points
*/
std::vector<Eigen::Vector3d> convert_into_tangent_space(const std::vector<Eigen::Vector3d>& pntList, const std::vector<size_t>& dpIdx);

/**
* @brief  Approximating each curve segments with arc
* @param  pntList    sequential points on curve
* @param  sepIdx     index of seperation points
* @return pairs of arc_mid points indexing and radius for each segment
*/
std::vector<std::pair<size_t, double>> arc_approximation(const std::vector<Eigen::Vector3d>& pntList, const std::vector<size_t>& sepIdx);

