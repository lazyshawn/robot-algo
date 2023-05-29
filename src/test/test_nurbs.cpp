#include <iostream>
#include <fstream>
#include <random>
#include <chrono>

#include "path_planning/nurbs.h"
#include "user_interface/user_interface.h"

void test_curve();
void test_surface();

int main(int argc, char** argv) {
  // test_curve();
  test_surface();

  return 0;
}

void test_surface() {
  const int numU = 3, numV = 3, orderU = 3, orderV = 3;
  NURBS_Surface surface(numU, numV, orderU, orderV);

  std::cout << "==> Initialization:" << std::endl;
  // 随机生成控制点
  std::cout << "Control points:" << std::endl;
  for (int i=0; i<numU; ++i) {
    Eigen::Matrix<double, 3, numV> pntMat = get_random_matrix(3, numV, 0, 10);
    std::cout << pntMat << "\n" << std::endl;
    for (int j=0; j<numV; ++j) {
      surface.ctrlPoints[i][j] = pntMat.col(j);
    }
  }
  // 设置控制点权重
  surface.weight = std::vector<std::vector<double>>(numU, std::vector<double>(numV, 1));

  // 设置节点向量
  surface.set_pinned_uniform_knots();
  std::cout << "knots:" << std::endl;
  for (int i=0; i<static_cast<int>(surface.curveU->knots.size()); ++i) {
    std::cout << surface.curveU->knots[i] << ", ";
  }
  std::cout << std::endl;
  for (int i=0; i<static_cast<int>(surface.curveV->knots.size()); ++i) {
    std::cout << surface.curveV->knots[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = surface.get_point(0.3,0.2);
  std::cout << point << std::endl;

  // 记录控制点
  std::ofstream ctrlPntFile("build/data/nurbs_surface_ctrlpoint", std::ios::trunc);
  int numu = surface.ctrlPoints.size(), numv = surface.ctrlPoints[0].size();
  ctrlPntFile << numu << std::endl << numv << std::endl;
  for (int i=0; i<numu; ++i) {
    for (int j=0; j<numv; ++j) {
      ctrlPntFile << surface.ctrlPoints[i][j].transpose() << std::endl;
    }
  }
  // 记录生成曲线
  std::ofstream curveFile("build/data/nurbs_surface_output", std::ios::trunc);
  // 采样点数
  double sepU = 20, sepV = 20;
  double du = (surface.curveU->knots[numu] - surface.curveU->knots[surface.curveU->degree]) / (sepU - 1);
  double dv = (surface.curveV->knots[numv] - surface.curveV->knots[surface.curveV->degree]) / (sepV - 1);
  curveFile << sepU << std::endl << sepV << std::endl;
  double u = surface.curveU->knots[surface.curveU->degree];
  for (int i=0; i<sepU; ++i) {
    double v = surface.curveV->knots[surface.curveV->degree];
    for (int j=0; j<sepV; ++j) {
      u = u > surface.curveU->knots[numu] ? surface.curveU->knots[numu] : u;
      v = v > surface.curveV->knots[numv] ? surface.curveV->knots[numv] : v;
      Eigen::Vector3d point = surface.get_point(u,v);
      curveFile << point.transpose() << std::endl;
      v += dv;
    }
    u += du;
  }
  // double u = curve.knots[curve.degree];
  // for (int i = 0; i < 100 + 1; ++i) {
  //   Eigen::Vector3d point = curve.get_point(u);
  //   u += du;
  //   curveFile << point.transpose() << std::endl;
  // }
}

void test_curve() {
  int num = 10, order = 3;
  NURBS_Curve curve(num, order);

  std::cout << "==> Initialization:" << std::endl;
  // 随机生成控制点
  Eigen::MatrixXd pntMat = get_random_matrix(3, num, 0, 10);
  std::cout << "Control points:\n" << pntMat << std::endl;
  for (int i=0; i<pntMat.cols(); ++i) {
    curve.ctrlPoints[i] = pntMat.col(i);
  }
  // 设置控制点权重
  curve.weight = std::vector<double>(curve.ctrlPoints.size(), 1);

  // 设定节点向量
  curve.set_pinned_uniform_knots();
  std::cout << "Knots:" << std::endl;
  for (int i=0; i<static_cast<int>(curve.knots.size()); ++i) {
    std::cout << curve.knots[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(8.001);
  std::cout << point << std::endl;

  // 记录控制点
  std::ofstream ctrlPntFile("build/data/nurbs_curve_ctrlpoint", std::ios::trunc);
  for (int i=0; i<pntMat.cols(); ++i) {
    ctrlPntFile << curve.ctrlPoints[i].transpose() << std::endl;
  }
  // 记录生成曲线
  std::ofstream curveFile("build/data/nurbs_curve_output", std::ios::trunc);
  double u = curve.knots[curve.degree];
  double du = (curve.knots[curve.ctrlPoints.size()] - curve.knots[curve.degree]) / 100;
  for (int i = 0; i < 100 + 1; ++i) {
    Eigen::Vector3d point = curve.get_point(u);
    u += du;
    curveFile << point.transpose() << std::endl;
  }
}

