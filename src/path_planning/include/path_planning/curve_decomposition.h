#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

class DiscreteTrajectory {
private:
  size_t cnt = 0;

public:
  enum type{NONE=0, CIRCLE=2, LINE=3};

  std::vector<Eigen::Vector3d> pntList;
  //! 轨迹上的位置点
  std::vector<Eigen::Isometry3d> pntPose;
  //! 过渡点姿态，比位置点数量少一
  std::vector<Eigen::Isometry3d> midPose;
  //! 圆弧半径
  std::vector<double> radii;
  //! 分割点索引
  std::vector<size_t> sepIdx;
  //! 圆弧中心点的索引
  std::vector<size_t> midIdx;

public:
  DiscreteTrajectory(){};
  ~DiscreteTrajectory(){};

  /**
  * @brief  加载数据点
  * @param  point 数据点
  */
  void load_trajectory(const std::vector<Eigen::Vector3d>& point);
  /**
  * @brief  将轨迹分割，保存分割点的索引
  * @param  err 分割误差
  */
  void perform_trajectory_decomposition(double linearErr = 8e-2, double curveErr = 8e-2);
  /**
  * @brief  计算离散点上的位姿
  */
  void calc_trajectory_pose(Eigen::Vector3d upper = {0,0,1});
  /**
  * @brief  清除数据
  */
  void clear();

};

Eigen::Vector3d circular_arc_fitting(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end);

std::vector<size_t> maximal_blurred_segment_split(const std::vector<Eigen::Vector3d>& pntList, double epsilon);

std::vector<Eigen::Vector3d> convert_into_tangent_space(const std::vector<Eigen::Vector3d>& pntlist, const std::vector<size_t>& dpIdx);

std::vector<std::pair<size_t, double>> arc_approximation(const std::vector<Eigen::Vector3d>& pntList, const std::vector<size_t>& sepIdx);

