#include "nurbs.h"
#include <iostream>

NURBS_Curve::NURBS_Curve(int num, int order_) {
  order = order_, degree = order - 1;
  ctrlPoints = std::vector<Eigen::Vector3d>(num);
  weight = std::vector<double>(num, 1);
  knots = std::vector<double>(num + order);
}

void NURBS_Curve::set_pinned_uniform_knots() {
  int num = static_cast<int>(ctrlPoints.size());
  double knotValue = 0.0;
  for (int i = 0; i < order; ++i) {
    knots[i] = knotValue;
  }
  for (int i = order; i < num; ++i) {
    knotValue++;
    knots[i] = knotValue;
  }
  knotValue++;
  for (int i = num; i < static_cast<int>(knots.size()); ++i) {
    knots[i] = knotValue;
  }
}

void NURBS_Curve::normalize_knots() {
  int maxKnot = *(knots.end() - 1);
  maxKnot = maxKnot == 0 ? 1 : maxKnot;
  for (int i=0; i<knots.size(); ++i) {
    knots[i] /= maxKnot;
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
  // 处理超出区间范围的参数值
  if (u > knots[ctrlPoints.size()]) {
    printf("-- Warning: u = %f is out of range [u_order, u_num], set to u_num.\n", u);
    u = knots[ctrlPoints.size()];
  } else if (u < knots[degree]) {
    printf("-- Warning: u = %f is out of range [u_order, u_num], set to u_order.\n", u);
    u = knots[degree];
  }

  return basis_function(idx, degree, u);
}

int NURBS_Curve::find_knot_span(double u) {
  int idx = 0;
  for (int i = 0; i < static_cast<int>(knots.size()); ++i) {
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
  return idx;
}

Eigen::Vector3d NURBS_Curve::get_point(double u) {
  // 确定参数所属的节点跨度区间
  int idx = find_knot_span(u);

  // 计算激活的控制点的贡献值 (order 个)
  double sum = 0.0;
  Eigen::Vector3d point{0,0,0};
  for (int i = idx; i > idx - order; --i) {
    double tmp = basis_function(i,u) * weight[i];
    Eigen::Vector3d tmpPnt = tmp * ctrlPoints[i];
    sum += tmp;
    point += tmpPnt;
  }

  return point / sum;
}

void NURBS_Curve::least_squares_fitting(std::vector<Eigen::Vector3d>& points) {
  const int n = points.size(), m = ctrlPoints.size() - 1;
  ctrlPoints[0] = points[0];
  ctrlPoints[m] = points[n-1];

  double sumSepDist = 0.0;
  std::vector<double> sepDist(n-1);
  for (int i = 1; i < n; ++i) {
    sepDist[i-1] = (points[i] - points[i - 1]).norm();
    sumSepDist += sepDist[i-1];
  }

  // 定义参数序列
  std::cout << "n = " << n << std::endl;
  std::vector<double> paraU(n, 0.0);
  std::cout << "sdf" << std::endl;
  paraU[n - 1] = 1;
  for (int i = 1; i < n-1; ++i) {
    paraU[i] = paraU[i-1] + sepDist[i]/sumSepDist;
  }

  std::vector<Eigen::Vector3d> res(n-1, {0,0,0});
  for (int i=1; i<n-1; ++i) {
    res[i] = points[i] - basis_function(0, paraU[i])*points[0] - basis_function(m,paraU[i]) * points[n-1];
  }

  Eigen::MatrixXd N(n-2,m-1), D(m-1,3), R(m-1,3);
  for (int i=0; i<n-2; ++i) {
    for (int j=0; j<m-1; ++j) {
      N(i,j) = basis_function(j+1,paraU[i+1]);
    }
  }
  // std::cout << N << std::endl;

  for (int i=0; i<m-1; ++i) {
    Eigen::Vector3d tmpPnt{0,0,0};
    for (int j=0; j<n-2; ++j) {
      tmpPnt += basis_function(i+1,paraU[j+1]) * res[j+1];
    }
    R.row(i) = tmpPnt.transpose();
  }
  // std::cout << R << std::endl;

  D = (N.transpose()*N).inverse() * R;
  std::cout << D << std::endl;
  for (int i=0; i<m-1; ++i) {
    ctrlPoints[i+1] = D.row(i).transpose();
  }
}


NURBS_Surface::NURBS_Surface(int numU_, int numV_, int orderU_, int orderV_) {
  curveU = std::make_unique<NURBS_Curve>(numU_, orderU_);
  curveV = std::make_unique<NURBS_Curve>(numV_, orderV_);

  ctrlPoints = std::vector<std::vector<Eigen::Vector3d>>(numU_, std::vector<Eigen::Vector3d>(numV_));
}

void NURBS_Surface::set_pinned_uniform_knots() {
  // U 方向的节点向量
  curveU->set_pinned_uniform_knots();
  // V 方向的节点向量
  curveV->set_pinned_uniform_knots();
}

Eigen::Vector3d NURBS_Surface::get_point(double u, double v) {
  Eigen::Vector3d point{0,0,0};

  // 确定参数所属的节点跨度区间
  int idxU = curveU->find_knot_span(u), idxV = curveV->find_knot_span(v);

  // 计算激活的控制点的贡献值 (order 个)
  double sum = 0.0;
  for (int i = idxU; i > idxU - curveU->order; --i) {
    for (int j = idxV; j > idxV - curveU->order; --j) {
      double tmp = curveU->basis_function(i, u) * curveV->basis_function(j, v) * weight[i][j];
      Eigen::Vector3d tmpPnt = tmp * ctrlPoints[i][j];
      sum += tmp;
      point += tmpPnt;
    }
  }
  return point / sum;
}


