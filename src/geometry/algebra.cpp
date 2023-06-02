#include "algebra.h"

#include <iostream>

double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint) {
  return norm.dot(target-viaPoint);
}

Eigen::Vector3d principal_component_analysis(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& axis, Eigen::Vector3d& com) {
  const int num = points.size(), dim = points[0].size();
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(num);
  Eigen::MatrixXd data = Eigen::MatrixXd(dim, num);

  for (int i = 0; i < num; ++i) data.col(i) = points[i];

  // 质心坐标
  com = data * ones / num;
  data.colwise() -= com;
  // 协方差矩阵
  Eigen::Matrix3d covMat = data * data.transpose() / (num - 1);

  // 奇异值分解
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(covMat, Eigen::ComputeFullU);
  Eigen::Matrix3d U = svd.matrixU();
  if (!axis.empty()) axis.clear();
  for (int i=0; i<U.cols(); ++i) {
    axis.emplace_back(U.col(i));
  }
  return svd.singularValues();
}

