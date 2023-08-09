#include "geometry/transformation.h"

Eigen::Matrix<double,4,4> liese3(Eigen::Vector<double,6> twist) {
  Eigen::Vector3d v = twist.head<3>(), w = twist.tail<3>();

  Eigen::Matrix<double,4,4> exponent = Eigen::Matrix<double,4,4>::Zero();
  exponent.topLeftCorner(3,3) = lieSO3(w);
  exponent.topRightCorner(3,1) = v;

  return exponent;
}

Eigen::Matrix3d lieSO3(Eigen::Vector<double,3> vec) {
  // 反对称矩阵
  return Eigen::Matrix3d({{0, -vec[2], vec[1]}, {vec[2], 0, -vec[0]}, {-vec[1], vec[0], 0}});
}

Eigen::Isometry3d lieSE3(Eigen::Vector<double,6> twist) {
  // 矩阵指数
  Eigen::Matrix<double,4,4> mat = liese3(twist).exp();
  return Eigen::Isometry3d(mat);
}

