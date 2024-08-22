#include "user_interface/user_interface.h"
#include "path_planning/curve_decomposition.h"

BezierCurve curve;
Eigen::MatrixXd linePnt;
// 保存轨迹点
std::ofstream lineFile("build/data/bezier/node_pnt");
// 保存样条曲线点
size_t numCurvePnt = 100;
std::ofstream curveFile("build/data/bezier/curve_pnt");
// 采样点
std::ofstream sampleFile("build/data/bezier/sample_pnt");

void corner_mode();
void vct_mode();

int main(int argc, char** argv) {
  vct_mode();

  return 0;
}

void corner_mode() {
  linePnt = get_random_matrix(3,3);
  // linePnt = Eigen::MatrixXd{{-1,0,0}, {0,0,0}, {0,1,0}};
  std::cout << linePnt << std::endl;

  Eigen::Vector3d beg = linePnt.row(0), mid = linePnt.row(1), end = linePnt.row(2);

  curve.construct_from_corner(beg, mid, end, 0.2);


  for (size_t i=0; i<linePnt.rows(); ++i) {
    lineFile << linePnt.row(i) << std::endl;
  }

  for (size_t i=0; i<numCurvePnt; ++i) {
    double u = 1.0 / (numCurvePnt-1) * i;
    curveFile << curve.get_point(u).transpose() << std::endl;
  }
}

void vct_mode() {
  linePnt = get_random_matrix(3,4);

  Eigen::Vector3d begPnt = linePnt.col(0), begVec = linePnt.col(1), endPnt = linePnt.col(2), endVec = linePnt.col(3);
  begVec.normalize();
  endVec.normalize();

  curve.construct_from_vct(begPnt, begVec, endPnt, endVec, 10);
  // 采样
  std::vector<double> param = curve.get_uniform_sample(0.2);

  for (size_t i=0; i<linePnt.cols()/2; ++i) {
    lineFile << linePnt.col(2*i).transpose() << " " << linePnt.col(2*i+1).normalized().transpose() << std::endl;
  }

  for (size_t i=0; i<numCurvePnt; ++i) {
    double u = 1.0 / (numCurvePnt-1) * i;
    curveFile << curve.get_point(u).transpose() << std::endl;
  }
  for (size_t i=0; i<param.size(); ++i) {
    std::cout << param[i] << std::endl;
    sampleFile << curve.get_point(param[i]).transpose() << std::endl;
  }

}

