#include "user_interface/user_interface.h"

Eigen::MatrixXd linePnt;

/**
* @brief  五次贝塞尔曲线平滑过渡
* @param  beg    过渡起点
* @param  mid    过渡尖点
* @param  end    过渡终点
* @return void
*/
void quintic_bezier_smooth(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end);

int main(int argc, char** argv) {
  std::cout << "hello world" << std::endl;

  linePnt = get_random_matrix(3,3);
  // linePnt = Eigen::MatrixXd{{-1,0,0}, {0,0,0}, {0,1,0}};
  std::cout << linePnt << std::endl;

  Eigen::Vector3d beg = linePnt.row(0), mid = linePnt.row(1), end = linePnt.row(2);
  quintic_bezier_smooth(beg, mid, end);


  std::ofstream lineFile("build/data/look_ahead/line_pnt");
  for (size_t i=0; i<linePnt.rows(); ++i) {
    lineFile << linePnt.row(i) << std::endl;
  }

  return 0;
}

void quintic_bezier_smooth(Eigen::Vector3d beg, Eigen::Vector3d mid, Eigen::Vector3d end) {
  // 确认夹角
  Eigen::Vector3d preDir = (beg - mid).normalized(), aftDir = (end - mid).normalized();
  double angle = std::acos(preDir.dot(aftDir));
  std::cout << "angle = " << angle*180/M_PI << std::endl;

  double nTheta = std::pow(angle, 0.9927) / 2.0769;
  std::cout << "nTheta = " << nTheta << std::endl;

  double delt = 0.2;
  double d = 32 * delt / (7*nTheta + 16) / std::sqrt(2+2*cos(angle));
  double c = d * nTheta;
  // 检查 Lt = 2c + d in (0, 0.5)

  std::vector<Eigen::Vector3d> p(6, mid);
  p[0] += preDir*(2*c + d);
  p[1] += preDir*(c + d);
  p[2] += preDir*(d);
  p[3] += aftDir*(d);
  p[4] += aftDir*(c + d);
  p[5] += aftDir*(2*c + d);

  // 绘制贝塞尔曲线
  std::ofstream curveFile("build/data/look_ahead/curve_pnt");
  double numPnt = 3;
  for (size_t i=0; i<numPnt; ++i) {
    double u = 1.0 / (numPnt-1) * i, mu = 1-u;
    Eigen::Vector3d pnt =
        mu * mu * mu * mu * mu * p[0] + 5 * u * mu * mu * mu * mu * p[1] +
        10 * u * u * mu * mu * mu * p[2] + 10 * u * u * u * mu * mu * p[3] +
        5 * u * u * u * u * mu * p[4] + u * u * u * u * u * p[5];
    curveFile << pnt.transpose() << std::endl;
  }
}

