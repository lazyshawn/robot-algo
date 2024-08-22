#include "path_planning/curve_decomposition.h"

std::vector<size_t> maximal_blurred_segment_split(const std::vector<Eigen::Vector3d>& pntList, double epsilon) {
  size_t beg = 0, end = 0, n = pntList.size();
  std::vector<std::pair<size_t, size_t>> mbs;

  Eigen::Vector3d origin, dir;
  double err = 0;
  while (err <= epsilon && end < n) {
    end++;
    // 更新拟合误差
    origin = pntList[beg];
    dir = (pntList[end] - pntList[beg]).normalized();
    // space_line_fitting(pntList.begin() + beg, pntList.begin() + end+1, center, norm);
    for (size_t i=beg+1; i<end; ++i) {
      Eigen::Vector3d cur = pntList[i] - origin;
      double dis = (cur - cur.dot(dir) * dir).norm();
      err = std::max(dis, err);
      // std::cout << "beg = " << beg << ", end = " << end << ", i = " << i <<  ", err = " << err << std::endl;
      if (err > epsilon) break;
    }
  }
  // 第一段 MBSv
  mbs.push_back(std::make_pair(beg, end-1));

  while (end < n-1) {
    // Remove a point at beginning of the B.S. while its width is greater than v
    while (err > epsilon) {
      beg++;
      // 更新离散线的参数
      origin = pntList[beg];
      dir = (pntList[end] - pntList[beg]).normalized();
      err = 0;
      for (size_t i=beg+1; i<end; ++i) {
        Eigen::Vector3d cur = pntList[i] - origin;
        double dis = (cur - cur.dot(dir) * dir).norm();
        err = std::max(dis, err);
        if (err > epsilon) break;
      }
    }
    // Insert a point at end of the B.S. while its width is less than v
    while (err <= epsilon) {
      end++;
      if (end == n) break;
      // 更新离散线的参数
      origin = pntList[beg];
      dir = (pntList[end] - pntList[beg]).normalized();
      // space_line_fitting(pntList.begin() + beg, pntList.begin() + end+1, center, norm);
      err = 0;
      for (size_t i=beg+1; i<end; ++i) {
        Eigen::Vector3d cur = pntList[i] - origin;
        double dis = (cur - cur.dot(dir) * dir).norm();
        err = std::max(dis, err);
      }
    }
    if (end < n) {
      mbs.push_back(std::make_pair(beg, end-1));
    }
  }

  // 处理最后一段包含最后一个点的情况
  end = n-1;
  while (err > epsilon) {
    beg++;
    // 更新离散线的参数
      origin = pntList[beg];
      dir = (pntList[end] - pntList[beg]).normalized();
    // space_line_fitting(pntList.begin() + beg, pntList.begin() + end+1, center, norm);
    err = 0;
    for (size_t i=beg+1; i<end; ++i) {
      Eigen::Vector3d cur = pntList[i] - origin;
      double dis = (cur - cur.dot(dir) * dir).norm();
      err = std::max(dis, err);
      if (err > epsilon) break;
    }
  }
  mbs.push_back(std::make_pair(beg, end));

  // for (size_t i=0; i<mbs.size(); ++i) {
  //   std::cout << mbs[i].first << ", " << mbs[i].second << std::endl;
  // }

  // 分段
  size_t pre = 0, cur = 0;
  std::vector<size_t> dpIdx = {0};
  while (cur < mbs.size()) {
    while (mbs[pre].second > mbs[cur].first) {
      cur++;
      if (cur == mbs.size()) break;
    }
    // 防止 cur,pre 相差 1 时无限循环
    if (cur == pre + 1) cur++;

    // std::cout << "DP:" << std::endl;
    // std::cout << mbs[pre].first << ", " << mbs[pre].second << std::endl;
    // std::cout << mbs[cur-1].first << ", " << mbs[cur-1].second << std::endl;
    // 角度最小的点记为 DP 点
    beg = mbs[cur-1].first;
    end = mbs[pre].second;
    double minAng = std::numeric_limits<double>::max();
    size_t dp = end;
    for (size_t i=beg+1; i<end; ++i) {
      // 计算夹角
      double angle = (pntList[end] - pntList[i]).normalized().dot(pntList[beg] - pntList[i]);
      if (angle < minAng) {
        minAng = angle;
        dp = i;
      }
    }

    // 处理相交区间内只有三个点的情况: 使相邻的两段直线夹角最小
    if (end - beg == 1) {
      Eigen::Vector3d preDir = (pntList[mbs[cur-1].first] - pntList[mbs[pre].first]).normalized();
      Eigen::Vector3d curDir = (pntList[mbs[pre].second] - pntList[mbs[cur-1].first]).normalized();
      Eigen::Vector3d aftDir = (pntList[mbs[cur-1].second] - pntList[mbs[pre].second]).normalized();

      dp = preDir.dot(curDir) > aftDir.dot(curDir) ? end : beg;
    }
    if (dp != dpIdx.back()) {
      dpIdx.push_back(dp);
    }
    pre = cur-1;
  }
  dpIdx.push_back(pntList.size()-1);

  return dpIdx;
}

std::vector<Eigen::Vector3d> convert_into_tangent_space(const std::vector<Eigen::Vector3d>& pntList, const std::vector<size_t>& dpIdx) {
  std::vector<Eigen::Vector3d> midPntCurve(dpIdx.size()-1);
  std::vector<Eigen::Vector3d> mpc(dpIdx.size()-1);

  // 初值
  double length = (pntList[dpIdx[1]] - pntList[dpIdx[0]]).norm()/2, theta = 0.0;
  Eigen::Vector3d posDir = (pntList[dpIdx[1]] - pntList[dpIdx[0]]).cross(pntList[dpIdx[2]] - pntList[dpIdx[1]]).normalized();
  mpc[0] = {length, 0, 0};

  for (size_t i=1; i<dpIdx.size()-1; ++i) {
    Eigen::Vector3d pre = pntList[dpIdx[i]] - pntList[dpIdx[i-1]];
    Eigen::Vector3d cur = pntList[dpIdx[i+1]] - pntList[dpIdx[i]];
    double curTheta = std::acos(pre.dot(cur) / pre.norm() / cur.norm());
    curTheta *= posDir.dot(pre.cross(cur)) > 0 ? 1 : -1;
    theta += curTheta;

    length += pre.norm()/2 + cur.norm()/2;

    mpc[i] = {length, theta, 0};
  }

  return mpc;
}


std::vector<std::pair<size_t, double>> arc_approximation(const std::vector<Eigen::Vector3d>& pntList, const std::vector<size_t>& sepIdx) {
  std::vector<std::pair<size_t, double>> result(sepIdx.size()-1);

  // 对逐段轨迹进行圆弧拟合
  for (size_t i=0; i<sepIdx.size()-1; ++i) {
    double radius = std::numeric_limits<double>::max(), minErr = radius;
    size_t minErrIdx = (sepIdx[i]+sepIdx[i+1])/2;
    Eigen::Vector3d begPnt = pntList[sepIdx[i]], endPnt = pntList[sepIdx[i+1]];

    // 遍历中间点
    for (size_t j=sepIdx[i]+1; j<sepIdx[i+1]-1; ++j) {
      // 用当前点作为圆弧中间点
      Eigen::Vector3d tmpCenter = triangular_circumcenter(begPnt, pntList[j], endPnt);
      double tmpRadius = (begPnt - tmpCenter).norm(), maxErr = 0.0;

      // 计算误差
      for (size_t k=sepIdx[i]+1; k<sepIdx[i+1]-1; ++k) {
        double err = std::fabs((pntList[k] - tmpCenter).norm() - tmpRadius);
        maxErr = std::max(err, maxErr);
      }

      // 更新最优的中间点
      if (maxErr < minErr) {
        minErrIdx = j;
        minErr = maxErr;
        radius = tmpRadius;
      }
    } // for (j)
    // 保存该段轨迹中需要的数据
    result[i] = {minErrIdx, radius};
  } // for (i)
  return result;
}

DiscreteTrajectory::DiscreteTrajectory(){
  this->clear();
}

DiscreteTrajectory::DiscreteTrajectory(const std::vector<Eigen::Vector3d>& points){
  load_trajectory(points);
}

void DiscreteTrajectory::calc_trajectory_pose(Eigen::Vector3d upper) {
  // Initialization
  radii = std::vector<double>(sepIdx.size() - 1);
  midIdx = std::vector<size_t>(sepIdx.size() - 1);
  pntPose = std::vector<Eigen::Isometry3d>(sepIdx.size());
  midPose = std::vector<Eigen::Isometry3d>(sepIdx.size() - 1);

  // 对逐段轨迹进行圆弧拟合 <圆弧点索引, 半径>
  std::vector<std::pair<size_t, double>> pair = arc_approximation(pntList, sepIdx);

  // center: 圆心, r: 半径，指向圆心, a: 轨迹弦长方向
  Eigen::Vector3d center, r, a;
  // d: 轨迹切线方向, n: 法线方向，垂直于前进方向，与 upper 成钝角, o: 局部圆弧平面内，轨迹右侧
  Eigen::Vector3d d, n, o;
  Eigen::Isometry3d tran = Eigen::Isometry3d::Identity();

  // 计算每一段的圆弧点和终点位姿
  for (size_t i=0; i<sepIdx.size()-1; ++i) {
    // 保存该段轨迹中需要的数据
    midIdx[i] = pair[i].first;
    radii[i] = pair[i].second;

    Eigen::Vector3d begPnt = pntList[sepIdx[i]], endPnt = pntList[sepIdx[i+1]];
    Eigen::Vector3d center = triangular_circumcenter(begPnt, pntList[midIdx[i]], endPnt);

    // 轨迹终点的位姿
    a = endPnt - pntList[sepIdx[i+1]-1], r = center - endPnt;
    d = r.cross(a.cross(r)).normalized(), n = upper.cross(d).cross(d).normalized(), o = d.cross(n);
    tran.linear() << o, d, n;
    tran.translation() = endPnt;
    pntPose[i+1] = tran;

    // 轨迹上圆弧点的位姿
    a = pntList[midIdx[i]] - pntList[midIdx[i]-1], r = center - pntList[midIdx[i]];
    d = r.cross(a.cross(r)).normalized(), n = upper.cross(d).cross(d).normalized(), o = d.cross(n);
    tran.linear() << o, d, n;
    tran.translation() = pntList[midIdx[i]];
    midPose[i] = tran;
  }

  // 第一个点的位姿
  center = triangular_circumcenter(pntList[sepIdx[0]], pntList[midIdx[0]], pntList[sepIdx[1]]);
  a = pntList[sepIdx[0]+1] - pntList[sepIdx[0]], r = center - pntList[sepIdx[0]];
  d = r.cross(a.cross(r)).normalized(), n = upper.cross(d).cross(d).normalized(), o = d.cross(n);
  tran.linear() << o, d, n;
  tran.translation() = pntList[sepIdx[0]];
  pntPose[0] = tran;

  // 姿态倾斜: 绕物体坐标系 y 轴旋转 pi/4
  for (size_t i=0; i<pntPose.size(); ++i) {
    pntPose[i].linear() = (Eigen::AngleAxisd(M_PI/4, pntPose[i].linear().col(1)) * pntPose[i].linear()).eval();
  }
  for (size_t i=0; i<midPose.size(); ++i) {
    midPose[i].linear() = (Eigen::AngleAxisd(M_PI/4, midPose[i].linear().col(1)) * midPose[i].linear()).eval();
  }
}

void DiscreteTrajectory::load_trajectory(const std::vector<Eigen::Vector3d>& points) {
  this->clear();

  // Initialize pntList
  pntList = std::vector<Eigen::Vector3d>(points.size());
  // Copy points to pntList
  for (size_t i=0; i<points.size(); ++i) {
    pntList[i] = points[i];
  }
}

void DiscreteTrajectory::perform_trajectory_decomposition(double linearErr, double curveErr) {
  // Blurred segment splition for midpoitn curve -> dominant points
  std::vector<size_t> dpIdx = maximal_blurred_segment_split(pntList, linearErr);
  // Get midpoint curve of the maximal blurred segment
  std::vector<Eigen::Vector3d> mpc =  convert_into_tangent_space(pntList, dpIdx);
  // Blurred segment splition for midpoitn curve
  std::vector<size_t> idxList = maximal_blurred_segment_split(mpc, curveErr);

  // Initialize sepIdx
  sepIdx = std::vector<size_t>(idxList.size(), dpIdx.back());
  // 圆弧和直线段分隔点的索引
  for (size_t i=0; i<idxList.size()-1; ++i) {
    sepIdx[i] = dpIdx[idxList[i]];
  }
}

void DiscreteTrajectory::clear() {
  pntList.clear();
  pntPose.clear();
  midPose.clear();
  radii.clear();
  sepIdx.clear();
  midIdx.clear();
}

void DiscreteTrajectory::export_trajectory(std::vector<Eigen::Vector3d>& nodePnt, std::vector<Eigen::Vector<double,7>>& arcInfo) {
  nodePnt = std::vector<Eigen::Vector3d>(sepIdx.size());
  arcInfo = std::vector<Eigen::Vector<double,7>>(midIdx.size());

  // 轨迹点坐标
  for (size_t i=0; i<sepIdx.size(); ++i) {
    nodePnt[i] = pntList[sepIdx[i]];
  }

  // 轨迹近似圆弧的信息
  for (size_t i=0; i<midIdx.size(); ++i) {
    Eigen::Vector3d beg = pntList[sepIdx[i]], mid = pntList[midIdx[i]], end = pntList[sepIdx[i+1]];
    // 计算圆心
    Eigen::Vector3d center = triangular_circumcenter(beg, mid, end);
    Eigen::Vector3d op0 = beg - center, op1 = mid - center, op2 = end - center;
    // 法向量
    Eigen::Vector3d normal = (op0.cross(op2)).normalized();
    // 圆心角
    double theta = std::acos(op0.dot(op2) / op0.norm() / op2.norm());

    // 根据 mid 是否在优弧上，
    if (normal.dot(op0.cross(op1)) < 0 || normal.dot(op1.cross(op2)) < 0) {
      // 修正法向量和圆心角
      normal *= -1;
      theta = 2*M_PI - theta;
    }

    // 圆心
    arcInfo[i].head(3) = center;
    // 半径
    arcInfo[i](3) = (beg - center).norm();
    // 正法向
    arcInfo[i].tail(3) = normal * theta;
  }
}

void DiscreteTrajectory::export_trajectory(std::list<Eigen::Vector3d> &nodePnt, std::list<Eigen::Vector<double, 7>> &arcInfo) {
  std::vector<Eigen::Vector3d> tmpNodePnt;
  std::vector<Eigen::Vector<double, 7>> tmpArcInfo;

  export_trajectory(tmpNodePnt, tmpArcInfo);

  nodePnt = std::list<Eigen::Vector3d>(tmpNodePnt.begin(), tmpNodePnt.end());
  arcInfo = std::list<Eigen::Vector<double, 7>>(tmpArcInfo.begin(), tmpArcInfo.end());
}

BezierCurve::BezierCurve() {
}

uint8_t BezierCurve::construct_from_vct(Eigen::Vector3d begPnt, Eigen::Vector3d begVec, Eigen::Vector3d endPnt, Eigen::Vector3d endVec, double bowHightErr) {
  begVec.normalize();
  endVec.normalize();

  // 切线夹角
  double angle = std::acos(begVec.dot(-endVec) / begVec.norm() / endVec.norm());
  // 系数
  double nTheta = std::pow(angle, 0.9927) / 2.0769;
  double d = 32 * bowHightErr / (7*nTheta + 16) / std::sqrt(2+2*cos(angle));
  double c = d * nTheta;
  // 检查 Lt = 2c + d in (0, 0.5)

  c = bowHightErr;
  // 确定控制点坐标
  ctrlPnt = std::vector<Eigen::Vector3d>(6);
  ctrlPnt[0] = begPnt;
  ctrlPnt[1] = begPnt + begVec * c;
  ctrlPnt[2] = begPnt + 2 * begVec * c;
  ctrlPnt[3] = endPnt - 2 * endVec * c;
  ctrlPnt[4] = endPnt - endVec * c;
  ctrlPnt[5] = endPnt;

  return 0;
}

uint8_t BezierCurve::construct_from_corner(Eigen::Vector3d begPnt, Eigen::Vector3d midPnt, Eigen::Vector3d endPnt, double bowHightErr) {
    // 确认夹角
  Eigen::Vector3d preDir = (begPnt - midPnt).normalized(), aftDir = (endPnt - midPnt).normalized();
  double angle = std::acos(preDir.dot(aftDir));

  double nTheta = std::pow(angle, 0.9927) / 2.0769;
  double d = 32 * bowHightErr / (7*nTheta + 16) / std::sqrt(2+2*cos(angle));
  double c = d * nTheta;
  // 检查 Lt = 2c + d in (0, 0.5)

  ctrlPnt = std::vector<Eigen::Vector3d>(6, midPnt);
  ctrlPnt[0] += preDir*(2*c + d);
  ctrlPnt[1] += preDir*(c + d);
  ctrlPnt[2] += preDir*(d);
  ctrlPnt[3] += aftDir*(d);
  ctrlPnt[4] += aftDir*(c + d);
  ctrlPnt[5] += aftDir*(2*c + d);

  return 0;
}

Eigen::Vector3d BezierCurve::get_point(double param) const {
    double u = param, mu = 1-u;
    Eigen::Vector3d pnt =
        mu * mu * mu * mu * mu * ctrlPnt[0] + 5 * u * mu * mu * mu * mu * ctrlPnt[1] +
        10 * u * u * mu * mu * mu * ctrlPnt[2] + 10 * u * u * u * mu * mu * ctrlPnt[3] +
        5 * u * u * u * u * mu * ctrlPnt[4] + u * u * u * u * u * ctrlPnt[5];

    return pnt;
}

void BezierCurve::discrete_arc_length(std::vector<double> &paraVector, double begPara, double endPara, double terminateCondition,
                                      double segmentLen, size_t numSegment) const {

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
    // 离散后的弦长之和
    double newChordLength = 0.0;
    // 子区间内的最大弦长
    double maxSubChordLength = 0.0;
    // 循环变量初始化
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
    double absDeltaLength = newChordLength - rawChordLength;
    double relDeltaLength = absDeltaLength / rawChordLength;
    if ((relDeltaLength > terminateCondition) || (segmentLen > 0 && (maxSubChordLength > segmentLen))) {
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

double BezierCurve::get_chord_length(const std::vector<double>& nodePara, std::vector<double>& cumuChordLength) const {
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

std::vector<double> BezierCurve::get_uniform_sample(const std::vector<double> &discretePara, const std::vector<double> &cumuChordLength,
    double length, double threshold) const {

    // 采样点对应的参数值
  std::vector<double> uniformParaVector;

  double curveLength = *(cumuChordLength.end() - 1);
  int numSegment = std::ceil(curveLength / length);
  // 分段长度
  double detLen = curveLength / numSegment, curChordLength = 0.0;
  size_t curSegmentIdx = 0;
  for (size_t i = 0; i < discretePara.size(); ++i) {
    if (cumuChordLength[i] > curChordLength) {
      uniformParaVector.emplace_back(discretePara[i-1]);
      curSegmentIdx++;
      // 间隔更接近给定值，但最后一段会更短
      // curChordLength = cumuChordLength[i-1] + detLen;
      // 整体间隔更均匀，但间隔可能大于给定值，超出的范围取决于曲线离散化程度
      curChordLength = curSegmentIdx * detLen;
      // std::cout << "idx = " << i - 1 << ", " << cumuChordLength[i - 1] << std::endl;
    }
  }
  uniformParaVector.emplace_back(*(discretePara.end()-1));
  return uniformParaVector;
}

std::vector<double> BezierCurve::get_uniform_sample(double length) {
  std::vector<double> paraVec;
  discrete_arc_length(paraVec, 0, 1, 1e-2, length*0.1);
  std::vector<double> cumuChordLength;
  get_chord_length(paraVec, cumuChordLength);
  std::vector<double> samplePara = get_uniform_sample(paraVec, cumuChordLength, length);
  return samplePara;
}
