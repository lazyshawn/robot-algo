#include "path_planning/nurbs.h"

#include <iostream>

NURBS_Curve::NURBS_Curve(int num, int order_) {
  order = order_, degree = order - 1;
  // ctrlPoints = std::vector<Eigen::Vector3d>(num);
  weight = std::vector<double>(num);
  scaledWeight = std::vector<double>(num);
  knots = std::vector<double>(num + order);
}

void NURBS_Curve::set_pinned_uniform_knots() {
  double knotValue = 0.0;
  for (int i=0; i<order; ++i) {
    knots[i] = knotValue;
  }
  for (int i=order; i<ctrlPoints.size(); ++i) {
    knotValue++;
    knots[i] = knotValue;
  }
  knotValue++;
  for (int i=ctrlPoints.size(); i<knots.size(); ++i) {
    knots[i] = knotValue;
  }
}

double NURBS_Curve::basis_function(int idx, int degree_, double u) {
  // 未生效的控制点
  if (knots[idx + degree_ + 1] < u) {
    return 0;
  }
  // 已失效的控制点
  else if (knots[idx] > u) {
    return 0;
  }

  // 终止条件
  if (degree_ == 1) {
    if (u == knots[idx + 1]) {
      return 1;
    } else if (u < knots[idx + 1]) {
      return (u - knots[idx]) / (knots[idx + 1] - knots[idx]);
    }
    return (knots[idx + 2] - u) / (knots[idx + 2] - knots[idx + 1]);
  }

  // 递归: N(i,n) = f(i,n)N(i,n-1) + g(i+1,n)N(i+1,n-1)
  double spanF = knots[idx + degree_] - knots[idx], spanG = knots[idx + 1 + degree_] - knots[idx + 1];
  // deem 0/0 = 0
  spanF = spanF == 0 ? 1 : spanF;
  spanG = spanG == 0 ? 1 : spanG;
  // f(i,n) = (u - k(i)) / (k(i+n) - k(i)), g(i,n) = (k(i+n) 0 u) / (k(i+n) - k(i))
  double f = (u - knots[idx]) / spanF, g = (knots[idx + 1 + degree_] - u) / spanG;

  return f * basis_function(idx, degree_ - 1, u) + g * basis_function(idx + 1, degree_ - 1, u);
}

double NURBS_Curve::basis_function(int idx, double u) {
  return basis_function(idx, degree, u);
}

Eigen::Vector3d NURBS_Curve::get_point(double u) {
  // 确定参数所属的节点跨度区间
  int idx = 0;
  for (int i=0; i<knots.size(); ++i) {
    if (knots[i] <= u && knots[i+1] > u) {
      idx = i;
      break;
    }
    // 已经到最后的节点
    if (*(knots.end() - 1) - knots[i] < 1e-5) {
      idx = i - 1;
      break;
    }
  }

  // 计算激活的控制点的贡献值 (order 个)
  double sum = 0.0;
  Eigen::Vector3d point{0,0,0};
  for (int i = 0; i < order; ++i) {
    double tmp = basis_function(idx - i,u) * weight[idx - i];
    // std::cout << "ctrlPoints idx = " << idx - i << ": " << tmp << std::endl;
    Eigen::Vector3d tmpPnt = tmp * ctrlPoints[idx - i];
    sum += tmp;
    point += tmpPnt;
  }

  return point / sum;
}

