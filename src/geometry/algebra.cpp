#include "algebra.h"

double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint) {
  return norm.dot(target-viaPoint);
}

