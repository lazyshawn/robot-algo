#include "path_planning/curve_decomposition.h"

Eigen::Vector3d triangular_circumcenter(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end) {
  Eigen::Vector3d a = beg - mid, b = end - mid;
  if (a.cross(b).squaredNorm() < 1e-12) {
    double inf = std::numeric_limits<double>::max();
    return {inf, inf, inf};
  }

  return (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2*(a.cross(b)).squaredNorm()) + mid;
}

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

