#include <iostream>
#include <fstream>
#include <random>
#include <chrono>

#include "path_planning/nurbs.h"
#include "user_interface/user_interface.h"

int main(int argc, char** argv) {
  int num = 10, order = 3;
  NURBS_Curve curve(num, order);

  std::cout << "==> Initialization:" << std::endl;
  // 随机生成控制点
  Eigen::MatrixXd pntMat = get_random_matrix(3, num, 0, 10);
  std::cout << "Control points:\n" << pntMat << std::endl;
  for (int i=0; i<pntMat.cols(); ++i) {
    curve.ctrlPoints.push_back(pntMat.col(i));
  }
  // 设置控制点权重
  curve.weight = std::vector<double>(curve.ctrlPoints.size(), 1);

  // 设定节点向量
  curve.set_pinned_uniform_knots();
  std::cout << "Knots:" << std::endl;
  for (int i=0; i<curve.knots.size(); ++i) {
    std::cout << curve.knots[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(1);
  std::cout << point << std::endl;

  std::ofstream pointFile("build/qkhull_points", std::ios::trunc);
  double u = curve.knots[curve.degree];
  double du = (curve.knots[curve.ctrlPoints.size()] - curve.knots[curve.degree]) / 100;
  for (int i = 0; i<100; ++i) {
    Eigen::Vector3d point = curve.get_point(u);
    u += du;
    pointFile << point.transpose() << std::endl;
  }


  return 0;
}
