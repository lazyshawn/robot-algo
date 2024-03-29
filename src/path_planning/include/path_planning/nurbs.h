/**
* @file   nurbs.h
* @brief  Non-Uniform Rational B-Splines
* @see    
*/
#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <queue>
#include <optional>

/**
* @brief  3D NURBS curve
*
* construct, fitting, uniformly sampling
*/
class NURBS_Curve {
public:
  //! 曲线维度, 阶数
  int degree, order;
  //! 控制点序列
  std::vector<Eigen::Vector3d> ctrlPoints;
  //! 权重系数, 默认全为1
  std::vector<double> weight;
  //! 节点向量
  std::vector<double> knots;
  //! 保存的参数序列
  std::vector<double> para;
  //! 实际控制点数量
  size_t activeCtrlPoints;

public:
  NURBS_Curve(){};
  /**
  * @brief  构造函数
  * @param  order_ (最高) NURBS 曲线阶数
  * @param  num    (最多)控制点数量
  */
  NURBS_Curve(int order_, int num = 0);
  NURBS_Curve(NURBS_Curve& other);
  NURBS_Curve& operator=(NURBS_Curve&& other);

  /**
  * @brief  设置均匀的节点向量，保证曲线经过首末点
  */
  void set_pinned_uniform_knots();

  /**
  * @brief  查找参数 u 所在的最后一个区间跨度
  * @param  u 参数值
  * @return 区间跨度起点的索引
  */
  int find_knot_span(double u) const;

  /**
  * @brief  NURBS 基函数
  * @param  idx 当前控制点序号
  * @param  u   控制参数
  * @return 参数 u 处第 i 个基函数值
  */
  double basis_function(int idx, double u) const;

  /**
  * @brief  获取参数为 u 时的拟合点
  * @param  u 参数值
  * @return 参数 u 对应的点的坐标
  */
  Eigen::Vector3d get_point(double u) const;

  /**
  * @brief  将两个参数值之间的曲线离散化为弦长
  * @param  [out] nodePara     离散化后节点的参数值
  * @param  begPara            曲线起点的参数值
  * @param  endPara            曲线终点的参数值
  * @param  numSegment         一次离散过程中的区间数量
  * @param  terminateCondition 终止条件，当曲线长度变化量小于该值时停止迭代
  * @param  segmentLen         离散弦长的阈值
  */
  void discrete_arc_length(std::vector<double> &nodePara, double begPara, double endPara, double terminateCondition = 1e-2,
                           double segmentLen = -1, size_t numSegment = 10) const;

  /**
  * @brief  根据离散的节点计算曲线上的弦长之和
  * @param  nodePara         离散节点对应的参数值向量
  * @param  cumuChordLength  累加到第 i 个节点处的弦长总和
  * @return 参数区间内曲线的总长度
  */
  double get_chord_length(const std::vector<double>& nodePara, std::vector<double>& cumuChordLength) const;

  /**
  * @brief  按照给定的曲线长度获取均匀采样点
  * @param  discretePara    离散化后的节点参数向量
  * @param  cumuChordLength 离散化后累加到节点处的弦长之和
  * @param  length          每段曲线的长度
  * @param  threshold       (unused)
  * @return 采样后节点对应的参数向量
  */
  std::vector<double> get_uniform_sample(double length, double threshold = 1e-1, double segmentLen = -1);
  std::vector<double> get_uniform_sample(const std::vector<double>& discretePara, const std::vector<double>& cumuChordLength,
                                         double length, double threshold = 1e-1) const;

  /**
  * @brief  利用 Nurbs 曲线进行最小二乘拟合，修改曲线的控制点和权重
  * @param  points 待拟合的点
  * @return 邻近点的参数值序列
  */
  std::vector<double> least_squares_fitting(const std::vector<Eigen::Vector3d>& points);

  /**
  * @brief  根据拟合点和拟合精度，确定最少的控制点
  * @param  points    待拟合点
  * @param  threshold 阈值
  * @return 拟合精度，平均误差
  */
  double auto_fitting(const std::vector<Eigen::Vector3d>& points, const double& threshold);
  double auto_fitting_binary(const std::vector<Eigen::Vector3d>& points, const double& threshold);

private:
  /**
  * @brief  将节点向量归一化 u[0,1]
  */
  void normalize_knots();

  /**
  * @brief  NURBS 基函数
  * @param  idx     当前控制点序号
  * @param  order_  基函数维度
  * @param  u       控制参数
  * @return 参数 u 处第 i 个基函数值
  */
  double basis_function(int idx, int degree_, double u) const;
};


/**
* @brief  3D NURBS face
*
* construct
*/
class NURBS_Surface {
public:
  std::unique_ptr<NURBS_Curve> curveU, curveV;
  std::vector<std::vector<Eigen::Vector3d>> ctrlPoints;
  std::vector<std::vector<double>> weight;

public:
  NURBS_Surface(int numU_, int numV_, int orderU_, int orderV_);

  /**
  * @brief  设置均匀的节点向量，保证曲面经过四个角点
  */
  void set_pinned_uniform_knots();

  /**
  * @brief  获取参数为 u 时的拟合点
  * @param  u 参数值
  * @return 参数 u 对应的点的坐标
  */
  Eigen::Vector3d get_point(double u, double v);
};
