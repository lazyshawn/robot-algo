#include "nurbs.h"
#include "iostream"

NURBS_Curve::NURBS_Curve(int order_, int num) {
  order = order_, degree = order - 1;
  if (num == 0) { num = order; }
  ctrlPoints = std::vector<Eigen::Vector3d>(num);
  activeCtrlPoints = ctrlPoints.size();
  weight = std::vector<double>(num, 1);
  knots = std::vector<double>(num + order);
}

NURBS_Curve::NURBS_Curve(NURBS_Curve& other) {
  size_t num = other.ctrlPoints.size();

  order = other.order, degree = other.degree;
  ctrlPoints = std::vector<Eigen::Vector3d>(num);
  activeCtrlPoints = ctrlPoints.size();
  weight = std::vector<double>(num, 1);
  knots = std::vector<double>(num + order);
}

NURBS_Curve& NURBS_Curve::operator=(NURBS_Curve&& other) {
  size_t num = other.ctrlPoints.size();

  order = other.order, degree = other.degree;
  ctrlPoints = std::vector<Eigen::Vector3d>(num);
  activeCtrlPoints = ctrlPoints.size();
  weight = std::vector<double>(num, 1);
  knots = std::vector<double>(num + order);
  return *this;
}

void NURBS_Curve::set_pinned_uniform_knots() {
  double knotValue = 0.0;
  for (size_t i = 0; i < order; ++i) {
    knots[i] = knotValue;
  }
  for (size_t i = order; i < activeCtrlPoints; ++i) {
    knotValue++;
    knots[i] = knotValue;
  }
  knotValue++;
  for (size_t i = activeCtrlPoints; i < activeCtrlPoints + order; ++i) {
    knots[i] = knotValue;
  }
  // 节点向量单位化
  normalize_knots();
}

void NURBS_Curve::normalize_knots() {
  // 实际考虑的节点长度
  int activeKnots = activeCtrlPoints + order;
  // 最后一个节点
  int maxKnot = *(knots.begin() + activeKnots - 1);
  maxKnot = maxKnot == 0 ? 1 : maxKnot;
  // 节点值归一化
  for (int i=0; i<activeKnots; ++i) {
    knots[i] /= maxKnot;
  }
}

double NURBS_Curve::basis_function(int idx, int degree_, double u) const {
  // 已失效的控制点
  if (knots[idx + degree_ + 1] < u) {
    return 0;
  }
  // 未生效的控制点
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

double NURBS_Curve::basis_function(int idx, double u) const {
  // 处理超出区间范围的参数值
  if (u > knots[activeCtrlPoints]) {
    u = knots[activeCtrlPoints];
  } else if (u < knots[degree]) {
    u = knots[degree];
  }

  return basis_function(idx, degree, u);
}

int NURBS_Curve::find_knot_span(double u) const{
  int left = order-1, right = activeCtrlPoints-1;
  int result = left;
  while (left <= right) {
    int mid = left + (right - left) / 2;

    if(knots[mid] < u) {
      // 更新解
      result = mid;
      left = mid +1;
    } else {
      right = mid - 1;
    }
  }
  return result;
}

Eigen::Vector3d NURBS_Curve::get_point(double u) const {
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

void NURBS_Curve::get_uniform_sample(double length, double threshold) {
  // 二分法求第一个大于采样间隔的点
  double left = 0, right = 1;
  Eigen::Vector3d pre = get_point(0);
  while (left < right) {
    double mid = left + (right - left) / 2;
    Eigen::Vector3d tmp = get_point(mid);
    double len = (tmp - pre).squaredNorm();
    // 终止条件
    if (fabs(len - length) < threshold) {
      std::cout << "pre = " << pre.transpose() << std::endl;
      std::cout << "tmp = " << tmp.transpose() << std::endl;
      break;
    } else if (len < length) {
      left = mid;
    } else {
      right = mid;
    }
  }
}

std::vector<double> NURBS_Curve::least_squares_fitting(const std::vector<Eigen::Vector3d>& points) {
  const int n = points.size(), m = activeCtrlPoints - 1;
  ctrlPoints[0] = points[0];
  ctrlPoints[m] = points[n-1];

  // 计算离散拟合点的弦长之和
  double sumSepDist = 0.0;
  std::vector<double> sepDist(n-1);
  for (int i = 0; i < n-1; ++i) {
    sepDist[i] = (points[i+1] - points[i]).norm();
    sumSepDist += sepDist[i];
  }

  // 按弦长比例计算参数序列
  std::vector<double> paraU(n, 0.0);
  paraU[n - 1] = 1;
  for (int i = 1; i < n-1; ++i) {
    paraU[i] = paraU[i-1] + sepDist[i-1]/sumSepDist;
  }

  // 中间点的贡献
  std::vector<Eigen::Vector3d> res(n, {0,0,0});
  for (int i=1; i<n-1; ++i) {
    res[i] = points[i] - basis_function(0, paraU[i])*points[0] - basis_function(m,paraU[i]) * points[n-1];
  }

  // 构建约束方程 (n,m 相差过大时，这些矩阵将是稀疏矩阵)
  // Eigen::MatrixXd N(n-2,m-1), D(m-1,3), R(m-1,3);
  Eigen::SparseMatrix<double> N(n-2,m-1), D(m-1,3), R(m-1,3);
  for (int i=0; i<n-2; ++i) {
    for (int j=0; j<m-1; ++j) {
      // N(i,j) = basis_function(j+1,paraU[i+1]);
      double basisFunc = basis_function(j+1,paraU[i+1]);
      if (basisFunc > 1e-5) {
        N.insert(i,j) = basisFunc;
      }
    }
  }

  for (int i=0; i<m-1; ++i) {
    Eigen::Vector3d tmpPnt{0,0,0};
    for (int j=0; j<n-2; ++j) {
      tmpPnt += basis_function(i+1,paraU[j+1]) * res[j+1];
    }
    // R.row(i) = tmpPnt.transpose();
    for (size_t j=0; j<3; ++j) {
      R.insert(i,j) = tmpPnt(j);
    }
  }

  // 计算控制点
  Eigen::SparseMatrix<double> NTN = N.transpose() * N;
  Eigen::SparseMatrix<double> ident(m-1,m-1);
  ident.setIdentity();
  // Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(NTN);
  D = solver.solve(ident) * R;
  // D = (N.transpose()*N).inverse() * R;
  for (int i=0; i<m-1; ++i) {
    ctrlPoints[i+1] = D.row(i).transpose();
  }
  return paraU;
}

double NURBS_Curve::auto_fitting(const std::vector<Eigen::Vector3d>& points, const double& threshold) {
  const size_t n = points.size();
  if (ctrlPoints.size() < n) {
    *this = NURBS_Curve(order, n);
  }
  para = std::vector<double>(n);

  // 二分法查找精度小于阈值的方式
  int left = order, right = n;
  std::pair<size_t, double> bestFit{n,std::numeric_limits<double>::infinity()};
  while (left <= right) {
    int mid = left + (right - left) / 2;
    printf("left = %d, mid = %d, right = %d\n", left, mid, right);
    activeCtrlPoints = mid;
    // 设置节点向量
    set_pinned_uniform_knots();
    // 最小二乘 NURBS 拟合
    para = least_squares_fitting(points);
    // 计算拟合精度
    double sum = 0;
    for (size_t i=0; i<para.size(); ++i) {
      Eigen::Vector3d tmp = get_point(para[i]);
      sum += (tmp - points[i]).norm();
    }
    sum /= n;
    // 更新最优的平均拟合精度
    if (sum < bestFit.second) {
      bestFit = std::make_pair(mid, sum);
    }
    printf("cur accuracy = %lf\n", sum);

    if (left == right) {
      break;
    } else if (sum > threshold) {
      left = mid + 1;
    } else {
      // 避免重复检验
      if (left == mid) break;
      right = mid;
    }
  } // binary search

  // 未能获得满足条件的拟合方式，使用遍历过程中效果最优的拟合
  if (bestFit.second > threshold && activeCtrlPoints != bestFit.first) {
    std::cout << "Cannot fitting. bestFit: " << bestFit.first << ", " << bestFit.second << std::endl;
    activeCtrlPoints = bestFit.first;
    // 设置节点向量
    set_pinned_uniform_knots();
    // 最小二乘 NURBS 拟合
    para = least_squares_fitting(points);
  }
  return bestFit.second;
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


