#include "geometry/algebra.h"

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
