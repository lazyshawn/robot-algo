#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

class NURBS_Curve {
public:
  int degree, order;
  std::vector<Eigen::Vector3d> ctrlPoints;
  std::vector<double> weight;
  std::vector<double> knots;
  std::vector<double> scaledWeight;

public:
  NURBS_Curve(int num, int order_);

  void set_pinned_uniform_knots();

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
  * @brief : 获取参数为 u 时的拟合点
  * @param : u - 参数值
  * @return: 
  */
  Eigen::Vector3d get_point(double u);
};


template <typename T>
class NURBS_Surface {
  std::vector<T> ctrlPoints;
  std::vector<double> knots;
};

