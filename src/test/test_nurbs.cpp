
#include "nurbs.h"
#include "user_interface.h"

void sort_points_along_direction(std::vector<Eigen::Vector3d>& pointVector, Eigen::Vector3d direction);

void test_curve();
void test_surface();
void test_fitting();
void test_auto_fitting();

// 记录控制点
void record_ctrl_points(const NURBS_Curve& curve);
// 记录拟合点
void record_fitting_points(const std::vector<Eigen::Vector3d>& fitPnt);
// 记录曲线
void record_curve(const NURBS_Curve& curve, int sampleNum);

int main(int argc, char** argv) {
  // test_curve();
  // test_surface();
  // test_fitting();
  test_auto_fitting();

  return 0;
}

void test_auto_fitting() {
  const int numFit = 8;
  NURBS_Curve curve(4);
  // 随机生成拟合点
  std::cout << "Fitting points:" << std::endl;
  Eigen::Matrix<double, 3, numFit> pntMat = get_random_matrix(3, numFit, 0, 10);
  std::vector<Eigen::Vector3d> fitPnt(numFit);
  for (int i=0; i<numFit; ++i) {
    fitPnt[i] = pntMat.col(i);
  }
  // 拟合点排序
  sort_points_along_direction(fitPnt, Eigen::Vector3d({1,0,0}));
  std::cout << pntMat << "\n" << std::endl;

  curve.auto_fitting(fitPnt, 0.00003);

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(0.5);
  std::cout << point << std::endl;

  // 记录控制点
  record_ctrl_points(curve);
  // 记录拟合点
  record_fitting_points(fitPnt);
  // 记录生成曲线
  record_curve(curve, 100);
}

void test_fitting() {
  NURBS_Curve curve(4, 5);
  const int numFit = 5;
  // 随机生成拟合点
  std::cout << "Fitting points:" << std::endl;
  Eigen::Matrix<double, 3, numFit> pntMat = get_random_matrix(3, numFit, 0, 10);
  std::vector<Eigen::Vector3d> fitPnt(numFit);
  for (int i=0; i<numFit; ++i) {
    fitPnt[i] = pntMat.col(i);
  }
  // 拟合点排序
  sort_points_along_direction(fitPnt, Eigen::Vector3d({1,0,0}));
  std::cout << pntMat << "\n" << std::endl;

  // 设置节点向量
  curve.set_pinned_uniform_knots();
  // 计算控制点
  curve.least_squares_fitting(fitPnt);

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(0.5);
  std::cout << point << std::endl;

  // 记录控制点
  record_ctrl_points(curve);
  // 记录拟合点
  record_fitting_points(fitPnt);
  // 记录生成曲线
  record_curve(curve, 100);
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
}

void test_curve() {
  int num = 10, order = 3;
  NURBS_Curve curve(order, num);

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
  record_ctrl_points(curve);
  // 记录生成曲线
  record_curve(curve, 100);
}

void sort_points_along_direction(std::vector<Eigen::Vector3d>& pointVector, Eigen::Vector3d direction) {
  // Sort based on the projection on direction
  std::sort(pointVector.begin(), pointVector.end(), [direction](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return p1.dot(direction) < p2.dot(direction);
  });
}

// 记录控制点
void record_ctrl_points(const NURBS_Curve& curve) {
  std::ofstream ctrlPntFile("build/data/nurbs_curve_ctrlpoint", std::ios::trunc);
  for (int i=0; i<curve.activeCtrlPoints; ++i) {
    ctrlPntFile << curve.ctrlPoints[i].transpose() << std::endl;
  }
}

// 记录拟合点
void record_fitting_points(const std::vector<Eigen::Vector3d>& fitPnt) {
  std::ofstream fitPntFile("build/data/nurbs_curve_fitpoint", std::ios::trunc);
  for (int i=0; i<static_cast<int>(fitPnt.size()); ++i) {
    fitPntFile << fitPnt[i].transpose() << std::endl;
  }
}

// 记录生成曲线
void record_curve(const NURBS_Curve& curve, int sampleNum) {
  std::ofstream curveFile("build/data/nurbs_curve_output", std::ios::trunc);
  double u = curve.knots[curve.degree];
  double du = (curve.knots[curve.activeCtrlPoints] - curve.knots[curve.degree]) / (sampleNum - 1);
  for (int i = 0; i < sampleNum; ++i) {
    Eigen::Vector3d point = curve.get_point(u);
    u += du;
    curveFile << point.transpose() << std::endl;
  }
}

