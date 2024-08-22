/**
 * @file   curve_decomposition.h
 * @brief  Decomposition of a curve into arcs and line segments based on
 *         dominant point detection
 */
#pragma once

#include <queue>
#include <vector>

#include "geometry/algebra.h"

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
  DiscreteTrajectory(const std::vector<Eigen::Vector3d> &points);
  ~DiscreteTrajectory(){};

  /**
   * @brief  加载数据点
   * @param  point 数据点
   */
  void load_trajectory(const std::vector<Eigen::Vector3d> &points);
  /**
   * @brief  将轨迹分割，保存分割点的索引
   * @param  linearErr    直线近似误差
   * @param  curveErr     曲率分割误差
   */
  void perform_trajectory_decomposition(double linearErr = 8e-2,
                                        double curveErr = 8e-2);
  /**
   * @brief  计算离散点和圆弧点上的位姿
   * @param  upper    轨迹上表面方向
   */
  void calc_trajectory_pose(Eigen::Vector3d upper = {0, 0, 1});
  /**
   * @brief  清除数据
   */
  void clear();
  /**
   * @brief  导出轨迹数据
   * @param  nodePnt    每段轨迹的节点坐标
   * @param  arcInfo    圆弧信息: <圆心位置>, <半径, 大于零生效>, <法向和圆心角>
   */
  void export_trajectory(std::vector<Eigen::Vector3d> &nodePnt, std::vector<Eigen::Vector<double, 7>> &arcInfo);
  void export_trajectory(std::list<Eigen::Vector3d> &nodePnt, std::list<Eigen::Vector<double, 7>> &arcInfo);
};

class BezierCurve {
public:
  //! 控制点
  std::vector<Eigen::Vector3d> ctrlPnt;

  BezierCurve();

  /**
   * @brief  使用端点和向量形式构建贝塞尔曲线
   * @param  begPnt         开始点
   * @param  begVec         开始点处的切向量
   * @param  endPnt         结束点
   * @param  endVec         结束点处的切向量
   * @param  bowHightErr    弓高误差
   * @return 执行状态的标志位
   */
  uint8_t construct_from_vct(Eigen::Vector3d begPnt, Eigen::Vector3d begVec,
                             Eigen::Vector3d endPnt, Eigen::Vector3d endVec,
                             double bowHightErr);
  uint8_t construct_from_corner(Eigen::Vector3d begPnt, Eigen::Vector3d midPnt,
                                Eigen::Vector3d endPnt, double bowHightErr);
  /**
   * @brief  计算参数值对应的空间点
   * @param  param    参数值
   * @return 参数值 param 对应的空间点坐标
   */
  Eigen::Vector3d get_point(double param) const;
  /**
   * @brief  曲线长度离散化
   * @param  length    采样的长度
   * @return 所有采样点对应的参数值组成的向量
   */
  void discrete_arc_length(std::vector<double> &paraVector, double begPara,
                           double endPara, double terminateCondition = 1e-2,
                           double segmentLen = -1, size_t numSegment = 10) const;
  /**
   * @brief  根据离散的节点计算曲线上的弦长之和
   * @param  nodePara         离散节点对应的参数值向量
   * @param  cumuChordLength  累加到第 i 个节点处的弦长总和
   * @return 参数区间内曲线的总长度
   */
  double get_chord_length(const std::vector<double> &nodePara,
                          std::vector<double> &cumuChordLength) const;
  /**
   * @brief  根据曲线长度均匀采样
   * @param  length    采样的长度
   * @return 所有采样点对应的参数值组成的向量
   */
  std::vector<double>
  get_uniform_sample(const std::vector<double> &discretePara,
                     const std::vector<double> &cumuChordLength, double length,
                     double threshold = 1e-1) const;
  std::vector<double> get_uniform_sample(double length);
};

/**
 * @brief  Split a curve into maximal blurred segments (MBS)
 * @param  pntList    sequential points on curve
 * @param  epsilon    maximun distance from a point to its MBS
 * @return index of dominant points which is the seperation for adjacent MBS
 */
std::vector<size_t>
maximal_blurred_segment_split(const std::vector<Eigen::Vector3d> &pntList,
                              double epsilon);

/**
 * @brief  Get mid points curve (MPC) for set of line segments from its tangent
 * space
 * @param  pntList    sequential points on curve
 * @param  dpIdx index of dominant points
 */
std::vector<Eigen::Vector3d>
convert_into_tangent_space(const std::vector<Eigen::Vector3d> &pntList,
                           const std::vector<size_t> &dpIdx);

/**
 * @brief  Approximating each curve segments with arc
 * @param  pntList    sequential points on curve
 * @param  sepIdx     index of seperation points
 * @return pairs of arc_mid points indexing and radius for each segment
 */
std::vector<std::pair<size_t, double>>
arc_approximation(const std::vector<Eigen::Vector3d> &pntList,
                  const std::vector<size_t> &sepIdx);
