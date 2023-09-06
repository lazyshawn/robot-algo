#pragma once

#include <eigen3/Eigen/Dense>
#include <list>

/* 
* @brief : 点到平面的有向距离
* @param : target: 目标点
* @param : norm: 平面的外法向量(需先单位化)
* @param : viaPoint: 平面任意一点
* @return: 有向距离
*/
double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint);

/* 
* @brief : 沿给定方向排序
* @param : 
* @return: 
*/
void sort_along_direction(std::vector<Eigen::Vector3d>& points, Eigen::Vector3d direction);

/* 
* @brief : 用给定向量构造正交基底
* @param : vec - 三维空间中一个向量
* @return: 
*/
Eigen::Matrix3d construct_unit_orthogonal_basis(Eigen::Vector3d vec);

/* 
* @brief : 主成分分析
* @param : points - n 个数据点坐标
* @param : axis - 主轴方向
* @param : com - 质心坐标
* @return: JacobiSVD<Matrix3d> - SVD 分解对象
*/
template <typename Container, typename T>
Eigen::JacobiSVD<Eigen::MatrixXd> pca_svd(const Container& points, T& com);

/* 
* @brief : 主成分分析
* @param : points - n 个数据点坐标
* @param : axis - 主轴方向
* @param : com - 质心坐标
* @return: Vector3d - 奇异值
*/
template <typename Container, typename T>
T principal_component_analysis(const Container& points, Container& axis, T& com);

/* 
* @brief : 主成分分析
* @param : points - n 个数据点坐标
* @param : axis - 主轴方向
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

