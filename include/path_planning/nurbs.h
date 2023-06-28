#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <queue>

class NURBS_Curve {
public:
  int degree, order;
  std::vector<Eigen::Vector3d> ctrlPoints;
  std::vector<double> weight;
  std::vector<double> knots;
  std::vector<double> para;
  // 实际控制点数量
  size_t activeCtrlPoints;

public:
  NURBS_Curve(){};
  /* 
  * @brief : 构造
  * @param : order_ - (最高) NURBS 曲线阶数
  * @param : num - (最多)控制点数量
  */
  NURBS_Curve(int order_, int num = 0);
  NURBS_Curve(NURBS_Curve& other);
  NURBS_Curve& operator=(NURBS_Curve&& other);

  /* 
  * @brief : 设置均匀的节点向量，保证曲线经过首末点
  * @param : 
  * @return: 
  */
  void set_pinned_uniform_knots();

  /* 
  * @brief : 查找参数 u 所在的最后一个区间跨度
  * @param : u - 参数值
  * @return: (int) - 区间跨度起点的索引
  */
  int find_knot_span(double u) const;

  /* 
  * @brief : NURBS 基函数
  * @param : idx - 当前控制点序号
  * @param : u - 控制参数
  * @return: 
  */
  double basis_function(int idx, double u) const;

  /* 
  * @brief : 获取参数为 u 时的拟合点
  * @param : u - 参数值
  * @return: 
  */
  Eigen::Vector3d get_point(double u) const;

  void discrete_arc_length(std::list<double>& nodePara);
  void discrete_arc_length(std::list<double>& nodePara, std::list<double>::iterator&& begIte, std::list<double>::iterator&& endIte);

  /* 
  * @brief : 获取曲线长度
  * @param : threshold - 曲线长度增量阈值
  * @return: 
  */
  double get_curve_length(double threshold = 1e-1);

  /* 
  * @brief : 均匀采样
  * @param : length - NURBS 曲线上的采样间隔
  * @return: para - 采样节点对应的参数值
  */
  void get_uniform_sample(std::vector<double>& para, double length, double threshold);

  /* 
  * @brief : 利用 Nurbs 曲线进行最小二乘拟合，修改曲线的控制点和权重
  * @param : points - 待拟合的点
  * @return: 
  */
  std::vector<double> least_squares_fitting(const std::vector<Eigen::Vector3d>& points);

  /* 
  * @brief : 根据拟合点和拟合精度，确定最少的控制点
  * @param : points - 待拟合点
  * @param : threshold - 阈值
  * @return: double - 拟合精度，平均误差
  */
  double auto_fitting(const std::vector<Eigen::Vector3d>& points, const double& threshold);
  double auto_fitting_binary(const std::vector<Eigen::Vector3d>& points, const double& threshold);

private:
  /* 
  * @brief : 将节点向量归一化 u[0,1]
  * @param : 
  * @return: 
  */
  void normalize_knots();

  /* 
  * @brief : NURBS 基函数
  * @param : idx - 当前控制点序号
  * @param : order_ - 基函数维度
  * @param : u - 控制参数
  * @return: 
  */
  double basis_function(int idx, int degree_, double u) const;
};


class NURBS_Surface {
public:
  std::unique_ptr<NURBS_Curve> curveU, curveV;
  std::vector<std::vector<Eigen::Vector3d>> ctrlPoints;
  std::vector<std::vector<double>> weight;

public:
  NURBS_Surface(int numU_, int numV_, int orderU_, int orderV_);

  void set_pinned_uniform_knots();

  /* 
  * @brief : 获取参数为 u 时的拟合点
  * @param : u - 参数值
  * @return: 
  */
  Eigen::Vector3d get_point(double u, double v);
};
