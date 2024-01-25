/**
* @file   algebra.hpp
* @brief  Template for basic algebra
*
* 常见代数运算的模板函数
*/
#pragma once

#include <eigen3/Eigen/Dense>

/**
* @brief  主成分分析
* @param  points 数据点坐标
* @param  com   质心坐标
* @return SVD 分解对象
*/
template <typename Container, typename T>
Eigen::JacobiSVD<Eigen::MatrixXd> pca_svd(const Container& points, T& com);

/**
* @brief  主成分分析
* @param  points        数据点坐标
* @param  [out] axis   主轴方向
* @param  [out] com    质心坐标
* @return 三个奇异值构成的向量
*/
template <typename Container, typename T>
T principal_component_analysis(const Container& points, Container& axis, T& com);

/**
* @brief  主成分分析
* @param  points  数据点坐标
* @param  axis   主轴方向
*/
template <typename Container1, typename Container2>
void principal_component_analysis(const Container1& points, Container2& axis);

/* *****************************************************************************
 * Template
 * ************************************************************************** */
// 主成分分析
// 返回 SVD 分解对象
template <typename Container, typename T>
Eigen::JacobiSVD<Eigen::MatrixXd> pca_svd(const Container& points, T& com) {
  const int num = points.size(), dim = com.size();
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(num);
  Eigen::MatrixXd data = Eigen::MatrixXd(dim, num);

  typename Container::const_iterator itePnt = points.begin();
  for (int i = 0; i < num; ++i) {
    data.col(i) = *itePnt;
    itePnt++;
  }

  // 质心坐标
  com = data * ones / num;
  data.colwise() -= com;
  // 协方差矩阵
  Eigen::MatrixXd covMat = data * data.transpose() / (num - 1);

  // 奇异值分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeFullU);
  return svd;
}

// 计算主轴方向、质心位置、奇异值
template <typename Container, typename T>
T principal_component_analysis(const Container& points, Container& axis, T& com) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd = pca_svd(points, com);
  // 计算主轴方向
  Eigen::Matrix3d U = svd.matrixU();
  if (!axis.empty()) axis.clear();
  for (int i=0; i<U.cols(); ++i) {
    axis.emplace_back(U.col(i));
  }
  // 返回奇异值
  return svd.singularValues();
}

// 仅计算主轴方向
template <typename Container1, typename Container2>
void principal_component_analysis(const Container1& points, Container2& axis) {
  Eigen::VectorXd com((*points.begin()).size());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd = pca_svd(points, com);
  // 计算主轴方向
  Eigen::Matrix3d U = svd.matrixU();
  if (!axis.empty()) axis.clear();
  for (int i=0; i<U.cols(); ++i) {
    axis.emplace_back(U.col(i));
  }
}

