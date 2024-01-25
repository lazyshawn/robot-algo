#include "geometry/algebra.h"
#include <iostream>

double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint) {
  return norm.dot(target-viaPoint);
}

void sort_along_direction(std::vector<Eigen::Vector3d>& points, Eigen::Vector3d direction) {
  // Sort based on the projection on direction
  std::sort(points.begin(), points.end(), [direction](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return p1.dot(direction) < p2.dot(direction);
  });
}

Eigen::Matrix3d construct_unit_orthogonal_basis(Eigen::Vector3d vec) {
  vec.normalize();
  // 找到一个非零的元素
  int idx = 0;
  for (size_t i=0; i<vec.size(); ++i) {
    if (std::fabs(vec[i]) > 1e-9) {
      idx = i;
      break;
    }
  }
  // vec 补子空间内的向量
  Eigen::Vector3d ortho(0,0,0);
  ortho[idx] = -vec[(idx+1)%3];
  ortho[(idx+1)%3] = vec[idx];
  ortho.normalize();
  // 构造基底
  Eigen::Matrix3d basis;
  basis << vec, ortho, vec.cross(ortho);
  return basis;
}

bool is_near_zero(double value, double ellipse) {
  return fabs(value) < ellipse;
}

double circle_fitting(const std::vector<Eigen::Vector3d>& samples, Eigen::Vector3d& center, double relErr) {
  // 最小二乘系数
  Eigen::MatrixXd A(samples.size(),4), y(samples.size(),1);
  Eigen::MatrixXd zero = Eigen::MatrixXd::Zero(samples.size(),1);
  for (size_t i=0; i<samples.size(); ++i) {
    A(i,0) = 2*samples[i][0];
    A(i,1) = 2*samples[i][1];
    A(i,2) = 2*samples[i][2];
    A(i,3) = 1;
    y(i) = samples[i].dot(samples[i]);
  }
  // 使用 SVD 分解求解最小二乘问题
  Eigen::BDCSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // 右奇异向量，最后一列即为 A 的零空间基底 (容易知道解是一维的)
  Eigen::Matrix4d V = svd.matrixV();

  // 圆平面法向量
  Eigen::Vector3d normal(V.col(3).data());
  normal.normalize();
  // 球心对应的最小二乘解
  Eigen::Vector4d fitAns = svd.solve(y);
  // 球半径
  double R = std::sqrt(fitAns(3) + fitAns(0)*fitAns(0) + fitAns(1)*fitAns(1) + fitAns(2)*fitAns(2));
  // 球心位置
  Eigen::Vector3d sphereCenter(fitAns.data());

  // 母线到法向量上的投影
  double projSum = 0.0;
  for (auto& pnt : samples) {
    projSum += (pnt - sphereCenter).dot(normal);
  }
  // 球心到圆心的距离
  projSum /= samples.size();
  // 圆心位置: 球心沿法线到圆心
  center = Eigen::Vector3d(fitAns.data()) + normal * projSum;
  // 圆半径
  double r = std::sqrt(R*R - projSum*projSum);

  // 采样点到圆心的距离与半径之差的均方差与半径的比例
  double sumErr = 0.0;
  for (auto& pnt : samples) {
    double radius = (pnt - center).norm();
    sumErr += std::fabs(radius - r);
  }
  sumErr /= samples.size() * r;

  // 误差大于阈值
  if (sumErr > relErr) {
    r *= -1;
    printf("Warnning: #circle_fitting(): relative error is %lf greater than %lf.\n", sumErr, relErr);
  }
  return r;
}

Eigen::Vector3d triangular_circumcenter(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end) {
  Eigen::Vector3d a = beg - mid, b = end - mid;
  if (a.cross(b).squaredNorm() < 1e-12) {
    double inf = std::numeric_limits<double>::max();
    return {inf, inf, inf};
  }

  return (a.squaredNorm()*b - b.squaredNorm()*a).cross(a.cross(b)) / (2*(a.cross(b)).squaredNorm()) + mid;
}

