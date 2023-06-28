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
      left = mid + 1;
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
  Eigen::Vector3d point{0, 0, 0};
  for (int i = idx; i > idx - order; --i) {
    double tmp = basis_function(i, u) * weight[i];
    Eigen::Vector3d tmpPnt = tmp * ctrlPoints[i];
    sum += tmp;
    point += tmpPnt;
  }

  return point / sum;
}

void NURBS_Curve::discrete_arc_length(std::vector<double>& paraVector, double begPara, double endPara, int numSegment, double terminateCondition) const {
  // 初始化节点列表
  std::list<double> paraList = std::list<double>({begPara, endPara});
  // 初始化访问队列
  std::queue<std::list<double>::iterator> iteQueue;
  iteQueue.push(paraList.begin());

  while (!iteQueue.empty()) {
    std::list<double>::iterator ite = iteQueue.front();
    iteQueue.pop();
    // * 细分前的弦长
    double beg = *ite, end = *std::next(ite);
    Eigen::Vector3d curNode = get_point(*ite), endNode = get_point(end);
    // 初始弦长
    double rawChordLength = (curNode - endNode).norm();

    // * 离散化
    double intervalLen = (end - beg) / numSegment;
    double newChordLength = 0.0;
    // 自区间内的最大弦长
    double maxSubChordLength = 0.0;
    end = beg;
    for (size_t i=0; i<numSegment; ++i) {
      beg = end;
      Eigen::Vector3d preNode = curNode;
      end += intervalLen;
      curNode = get_point(end);
      double curChordLength = (curNode - preNode).norm();
      newChordLength += curChordLength;
      maxSubChordLength = std::max(maxSubChordLength, curChordLength);
    }

    // * 更新区间节点
    // 参数区间内曲线长度因细分新增的变化量 (绝对/相对)
    double deltaLength = (newChordLength - rawChordLength)/rawChordLength;
    // double deltaLength = newChordLength - rawChordLength;
    if (deltaLength > terminateCondition) {
      std::list<double>::iterator tmpIte = ite;
      // 重新将区间起点加入访问队列
      iteQueue.push(tmpIte);
      double cur = *ite;
      for (size_t i=1; i<numSegment; ++i) {
        // 插入新的节点
        cur += intervalLen;
        paraList.insert(std::next(tmpIte), cur);
        // 将新的节点加入访问队列
        tmpIte++;
        iteQueue.push(tmpIte);
      }
    } else {
      std::list<double>::iterator tmpIte = ite;
      double cur = *ite;
      for (size_t i=1; i<numSegment; ++i) {
        // 插入新的节点
        cur += intervalLen;
        paraList.insert(std::next(tmpIte), cur);
        tmpIte++;
      }
    } // if (newChordLength - rawChordLength)

  } // while (!iteQueue.empty())

  // 将节点列表转化为向量形式
  paraVector = std::vector<double>(std::make_move_iterator(paraList.begin()), std::make_move_iterator(paraList.end()));

} // discrete_arc_length()

double NURBS_Curve::get_chord_length(const std::vector<double>& nodePara, std::vector<double>& cumuChordLength) const {
  cumuChordLength.clear();
  cumuChordLength.reserve(nodePara.size());

  double length = 0.0;
  Eigen::Vector3d end = get_point(*nodePara.begin());
  for (auto& ite : nodePara) {
    Eigen::Vector3d beg = end;
    end = get_point(ite);
    length += (beg-end).norm();
    cumuChordLength.emplace_back(length);
  }
  return length;
}

std::vector<double> NURBS_Curve::get_uniform_sample(
    const std::vector<double>& paraVec, const std::vector<double>& cumuChordLength, double length, double threshold) const{
  // 采样点对应的参数值
  std::vector<double> uniformParaVector;

  double curveLength = *(cumuChordLength.end() - 1);
  int numSegment = std::ceil(curveLength / length);
  // 分段长度
  double detLen = curveLength / numSegment, curChordLength = 0.0;
  size_t curSegmentIdx = 0;
  for (size_t i = 0; i < paraVec.size(); ++i) {
    if (cumuChordLength[i] > curChordLength) {
      uniformParaVector.emplace_back(paraVec[i-1]);
      curSegmentIdx++;
      // 间隔更接近给定值，但最后一段会更短
      // curChordLength = cumuChordLength[i-1] + detLen;
      // 整体间隔更均匀，但间隔可能大于给定值，超出的范围取决于曲线离散化程度
      curChordLength = curSegmentIdx * detLen;
      // std::cout << "idx = " << i - 1 << ", " << cumuChordLength[i - 1] << std::endl;
    }
  }
  uniformParaVector.emplace_back(*(paraVec.end()-1));
  return uniformParaVector;
}

void NURBS_Curve::get_uniform_sample(std::vector<double> &paraVec, double length, double threshold) {
  // 节点处的累积弦长
  std::vector<double> cumulativeLength(1e5, 0.0);
  // * 参数区间离散化
  double curveLength = std::numeric_limits<double>::infinity(), sepPara = 1.0;
  // 间隔段数
  int sepNum = 100;
  // 计算总长度
  for (size_t iteration = 0; iteration < 6; ++iteration) {
    double curPara = 0, curLength = 0.0;
    Eigen::Vector3d cur = get_point(curPara), pre;
    sepPara = 1.0 / sepNum;

    // 计算曲线长度
    for (size_t i = 0; i < sepNum; ++i) {
      pre = cur;
      curPara += sepPara;
      cur = get_point(curPara);
      double len = (cur - pre).norm();
      curLength += len;
      cumulativeLength[i + 1] = curLength;
    } // for (sepNum)

    std::cout << "cur len = " << curLength << std::endl;
    // 终止条件
    if (std::fabs(curveLength - curLength) < threshold) {
      curveLength = curLength;
      break;
    }

    // 更新循环变量
    curveLength = curLength;
    sepNum *= 10;
  } // for (iteration)

  // * 查找分隔点对应的参数
  // 分段数
  int num = std::ceil(curveLength / length);
  // 分段长度
  double detLen = curveLength / num;
  paraVec = std::vector<double>();
  paraVec.reserve(num + 1);

  size_t curSegmentIdx = 0;
  for (size_t i = 0; i < sepNum; ++i) {
    if (cumulativeLength[i] > curSegmentIdx * detLen) {
      paraVec.emplace_back((i - 1) * sepPara);
      curSegmentIdx++;
      // std::cout << "idx = " << i - 1 << ", " << cumulativeLength[i - 1] << std::endl;
    }
  }
  paraVec.emplace_back(1);
} // get_uniform_sample()

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
  for (int i = 1; i < n - 1; ++i) {
    paraU[i] = paraU[i - 1] + sepDist[i - 1] / sumSepDist;
  }

  // 中间点的贡献
  std::vector<Eigen::Vector3d> res(n, {0, 0, 0});
  for (int i = 1; i < n - 1; ++i) {
    res[i] = points[i] - basis_function(0, paraU[i]) * points[0] -
             basis_function(m, paraU[i]) * points[n - 1];
  }

  // 构建约束方程 (n,m 相差过大时，这些矩阵将是稀疏矩阵)
  // Eigen::MatrixXd N(n-2,m-1), D(m-1,3), R(m-1,3);
  Eigen::SparseMatrix<double> N(n - 2, m - 1), D(m - 1, 3), R(m - 1, 3);
  for (int i = 0; i < n - 2; ++i) {
    for (int j = 0; j < m - 1; ++j) {
      // N(i,j) = basis_function(j+1,paraU[i+1]);
      double basisFunc = basis_function(j + 1, paraU[i + 1]);
      if (basisFunc > 1e-5) {
        N.insert(i, j) = basisFunc;
      }
    }
  }

  for (int i = 0; i < m - 1; ++i) {
    Eigen::Vector3d tmpPnt{0, 0, 0};
    for (int j = 0; j < n - 2; ++j) {
      tmpPnt += basis_function(i + 1, paraU[j + 1]) * res[j + 1];
    }
    // R.row(i) = tmpPnt.transpose();
    for (size_t j = 0; j < 3; ++j) {
      R.insert(i, j) = tmpPnt(j);
    }
  }

  // 计算控制点
  Eigen::SparseMatrix<double> NTN = N.transpose() * N;
  Eigen::SparseMatrix<double> ident(m - 1, m - 1);
  ident.setIdentity();
  // Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(NTN);
  D = solver.solve(ident) * R;
  // D = (N.transpose()*N).inverse() * R;
  for (int i = 0; i < m - 1; ++i) {
    ctrlPoints[i + 1] = D.row(i).transpose();
  }
  return paraU;
}

double NURBS_Curve::auto_fitting(const std::vector<Eigen::Vector3d> &points,
                                 const double &threshold) {
  const size_t n = points.size();
  if (ctrlPoints.size() < n) {
    *this = NURBS_Curve(order, n);
  }
  para = std::vector<double>(n);

  // 二分法查找精度小于阈值的方式
  std::pair<size_t, double> bestFit{n, std::numeric_limits<double>::infinity()};
  for (int mid = order; mid < n + 1; ++mid) {
    printf("mid = %d\n", mid);
    activeCtrlPoints = mid;
    // 设置节点向量
    set_pinned_uniform_knots();
    // 最小二乘 NURBS 拟合
    para = least_squares_fitting(points);
    // 计算拟合精度
    double sum = 0;
    for (size_t i = 0; i < para.size(); ++i) {
      Eigen::Vector3d tmp = get_point(para[i]);
      sum += (tmp - points[i]).norm();
    }
    sum /= n;
    printf("cur accuracy = %lf\n", sum);
    // 更新最优的平均拟合精度
    if (sum < threshold) {
      bestFit = std::make_pair(mid, sum);
      break;
    } else if (sum < bestFit.second) {
      bestFit = std::make_pair(mid, sum);
    }
  }

  // 未能获得满足条件的拟合方式，使用遍历过程中效果最优的拟合
  if (bestFit.second > threshold && activeCtrlPoints != bestFit.first) {
    std::cout << "Cannot fitting. bestFit: " << bestFit.first << ", "
              << bestFit.second << std::endl;
    activeCtrlPoints = bestFit.first;
    // 设置节点向量
    set_pinned_uniform_knots();
    // 最小二乘 NURBS 拟合
    para = least_squares_fitting(points);
  }
  return bestFit.second;
}

double
NURBS_Curve::auto_fitting_binary(const std::vector<Eigen::Vector3d> &points,
                                 const double &threshold) {
  const size_t n = points.size();
  if (ctrlPoints.size() < n) {
    *this = NURBS_Curve(order, n);
  }
  para = std::vector<double>(n);

  // 二分法查找精度小于阈值的方式
  int left = order, right = n;
  std::pair<size_t, double> bestFit{n, std::numeric_limits<double>::infinity()};
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
    for (size_t i = 0; i < para.size(); ++i) {
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
      if (left == mid)
        break;
      right = mid;
    }
  } // binary search

  // 未能获得满足条件的拟合方式，使用遍历过程中效果最优的拟合
  if (bestFit.second > threshold && activeCtrlPoints != bestFit.first) {
    std::cout << "Cannot fitting. bestFit: " << bestFit.first << ", "
              << bestFit.second << std::endl;
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

  ctrlPoints = std::vector<std::vector<Eigen::Vector3d>>(
      numU_, std::vector<Eigen::Vector3d>(numV_));
}

void NURBS_Surface::set_pinned_uniform_knots() {
  // U 方向的节点向量
  curveU->set_pinned_uniform_knots();
  // V 方向的节点向量
  curveV->set_pinned_uniform_knots();
}

Eigen::Vector3d NURBS_Surface::get_point(double u, double v) {
  Eigen::Vector3d point{0, 0, 0};

  // 确定参数所属的节点跨度区间
  int idxU = curveU->find_knot_span(u), idxV = curveV->find_knot_span(v);

  // 计算激活的控制点的贡献值 (order 个)
  double sum = 0.0;
  for (int i = idxU; i > idxU - curveU->order; --i) {
    for (int j = idxV; j > idxV - curveU->order; --j) {
      double tmp = curveU->basis_function(i, u) * curveV->basis_function(j, v) *
                   weight[i][j];
      Eigen::Vector3d tmpPnt = tmp * ctrlPoints[i][j];
      sum += tmp;
      point += tmpPnt;
    }
  }
  return point / sum;
}
