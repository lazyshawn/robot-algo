#pragma once

#include <vector>
#include <memory>
#include <eigen3/Eigen/Dense>

class NURBS_Curve {
public:
  int degree, order;
  std::vector<Eigen::Vector3d> ctrlPoints;
  std::vector<double> weight;
  std::vector<double> knots;
  // 实际控制点数量
  int activeCtrlPoints;

public:
  NURBS_Curve(){};
  /* 
  * @brief : 构造
  * @param : num - (最多)控制点数量
  * @param : order_ - (最高) NURBS 曲线阶数
  */
  NURBS_Curve(int num, int order_);

  /* 
  * @brief : 设置均匀的节点向量，保证曲线经过首末点
  * @param : 
  * @return: 
  */
  void set_pinned_uniform_knots();

  /* 
  * @brief : 将节点向量归一化 u[0,1]
  * @param : 
  * @return: 
  */
  void normalize_knots();

  /* 
  * @brief : 获取参数为 u 时的拟合点
  * @param : u - 参数值
  * @return: 
  */
  Eigen::Vector3d get_point(double u) const;

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
  * @return: 
  */
  void auto_fitting(const std::vector<Eigen::Vector3d>& points, const double& threshold);

private:
  /* 
  * @brief : 查找参数 u 所在的最后一个区间跨度
  * @param : u - 参数值
  * @return: (int) - 区间跨度起点的索引
  */
  int find_knot_span(double u) const;

  /* 
  * @brief : NURBS 基函数
  * @param : idx - 当前控制点序号
  * @param : order_ - 基函数维度
  * @param : u - 控制参数
  * @return: 
  */
  double basis_function(int idx, int degree_, double u) const;
  double basis_function(int idx, double u) const;
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
