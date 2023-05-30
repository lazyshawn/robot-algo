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

public:
  NURBS_Curve(){};
  /* 
  * @brief : 构造
  * @param : num - 控制点数量
  * @param : order_ - NURBS 曲线阶数
  */
  NURBS_Curve(int num, int order_);

  void set_pinned_uniform_knots();

  /* 
  * @brief : 将节点向量单位化 [0,1]
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
  double basis_function(int idx, int degree_, double u);

  double basis_function(int idx, double u);

  /* 
  * @brief : 查找参数 u 所在的最后一个区间跨度
  * @param : u - 参数值
  * @return: (int) - 区间跨度起点的索引
  */
  int find_knot_span(double u);
  /* 
  * @brief : 获取参数为 u 时的拟合点
  * @param : u - 参数值
  * @return: 
  */
  Eigen::Vector3d get_point(double u);

  /* 
  * @brief : 利用 Nurbs 曲线进行最小二乘拟合，修改曲线的控制点和权重
  * @param : points - 待拟合的控制点
  * @return: 
  */
  void least_squares_fitting(std::vector<Eigen::Vector3d>& points);
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
