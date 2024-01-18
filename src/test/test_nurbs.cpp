
#include "path_planning/nurbs.h"
#include "user_interface/user_interface.h"

void sort_points_along_direction(std::vector<Eigen::Vector3d>& pointVector, Eigen::Vector3d direction);

void test_curve();
void test_surface();
void test_fitting();
void test_auto_fitting();

// 记录控制点
void record_ctrl_points();
// 记录拟合点
void record_fitting_points();
// 记录曲线
void record_curve(int sampleNum = 100);

// 随机生成的控制点或拟合点数量
const int numPnts = 100;
NURBS_Curve curve(4);
Eigen::MatrixXd pntMat = get_random_matrix(3, numPnts, 0, 10);
std::vector<Eigen::Vector3d> fitPnt(numPnts);

int main(int argc, char** argv) {
  // test_curve();
  // test_surface();
  // test_fitting();
  test_auto_fitting();

  // 记录控制点
  record_ctrl_points();
  // 记录拟合点
  record_fitting_points();
  // 记录生成曲线
  record_curve();

  return 0;
}

void test_auto_fitting() {
  // 从文件读取拟合点数据
  read_eigen_from_file("data/cluster.txt", pntMat);
  pntMat.transposeInPlace();
  // fitPnt = std::vector<Eigen::Vector3d>(pntMat.cols());

  std::cout << "Fitting points:" << std::endl;
  std::cout << pntMat << "\n" << std::endl;

  // 以向量格式保存拟合点
  fitPnt = std::vector<Eigen::Vector3d>(pntMat.cols());
  for (int i=0; i<pntMat.cols(); ++i) {
    fitPnt[i] = pntMat.col(i);
  }

  // 拟合点排序
  // sort_points_along_direction(fitPnt, Eigen::Vector3d({1,0,0}));

  // 最小二乘拟合
  curve.auto_fitting(fitPnt, 3);
  
  // 均匀采样
  std::vector<double> samplePara = curve.get_uniform_sample(1);
  std::cout << "num of sample points: " << samplePara.size() << std::endl;

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(0.5);
  std::cout << point << std::endl;
}

void test_fitting() {
  // 随机生成拟合点
  std::cout << "Fitting points:" << std::endl;
  for (int i=0; i<pntMat.cols(); ++i) {
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
  std::cout << "Control points:\n" << pntMat << std::endl;
  curve = NURBS_Curve(4, numPnts);
  for (int i=0; i<numPnts; ++i) {
    curve.ctrlPoints[i] = pntMat.col(i);
    fitPnt[i] = pntMat.col(i);
  }
  // 设置控制点权重
  curve.weight = std::vector<double>(curve.ctrlPoints.size(), 1);
  // 设定节点向量
  curve.set_pinned_uniform_knots();

  std::cout << "\n===> Get points on curve:" << std::endl;
  Eigen::Vector3d point = curve.get_point(0.5);
  std::cout << "point at u: " << point.transpose() << std::endl;

  std::vector<double> paraVec;
  Timer timer;
  timer.start();
  // curve.get_uniform_sample(paraVec, 20, 0.2);
  // std::cout << "time = " << timer.ms_since_starting() << std::endl;;

  timer.record();
  curve.discrete_arc_length(paraVec, 0, 1);
  std::vector<double> cumuChordLength;
  std::cout << "\nlength = " << curve.get_chord_length(paraVec, cumuChordLength) << std::endl;
  std::cout << "time = " << timer.ms_since_last() << std::endl;;
  std::vector<double> samplePara = curve.get_uniform_sample(paraVec, cumuChordLength, 10);
}

void sort_points_along_direction(std::vector<Eigen::Vector3d>& pointVector, Eigen::Vector3d direction) {
  // Sort based on the projection on direction
  std::sort(pointVector.begin(), pointVector.end(), [direction](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    return p1.dot(direction) < p2.dot(direction);
  });
}

// 记录控制点
void record_ctrl_points() {
  std::ofstream ctrlPntFile("build/data/nurbs_curve_ctrlpoint", std::ios::trunc);
  for (int i=0; i<curve.activeCtrlPoints; ++i) {
    ctrlPntFile << curve.ctrlPoints[i].transpose() << std::endl;
  }
}

// 记录拟合点
void record_fitting_points() {
  std::ofstream fitPntFile("build/data/nurbs_curve_fitpoint", std::ios::trunc);
  for (int i=0; i<static_cast<int>(fitPnt.size()); ++i) {
    fitPntFile << fitPnt[i].transpose() << std::endl;
  }
}

// 记录生成曲线
void record_curve(int sampleNum) {
  std::ofstream curveFile("build/data/nurbs_curve_output", std::ios::trunc);
  double u = curve.knots[curve.degree];
  double du = (curve.knots[curve.activeCtrlPoints] - curve.knots[curve.degree]) / (sampleNum - 1);
  for (int i = 0; i < sampleNum; ++i) {
    Eigen::Vector3d point = curve.get_point(u);
    u += du;
    curveFile << point[0] << " " << point[1] << " " << point[2] << std::endl;
  }
}

