#include "algebra.h"

double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint) {
  return norm.dot(target-viaPoint);
}

void sort_along_direction(std::vector<Eigen::Vector3d>& points, Eigen::Vector3d direction) {
  // Sort based on the projection on direction
  std::sort(points.begin(), points.end(), [direction](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return p1.dot(direction) < p2.dot(direction);
  });
}

