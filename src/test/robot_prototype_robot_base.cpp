#include "robot_prototype/robot_base.h"

const double pi = 3.1415926535897932384626433832795028841971693993751058209;

std::string fname("data/config/fanuc_m20id25_scan.json");
robotBase robot;

int main(int argc, char** argv) {
  // 读取配置文件
  if (!robot.load_config(fname)) {
    std::cout << "Load robot config failed." << std::endl;
    return -1;
  }

  // 求解正运动学
  // std::vector<double> theta({0,-M_PI/2, 0,0,M_PI/2,0});
  Eigen::Isometry3d tran;
  std::vector<double> theta({-5.719, 36.739, -40.95, 0, -49.053, -72.278});
  // std::vector<double> theta({0.306, 48.406, -32.913, -0.021, -57.042, 157.207});
  // std::vector<double> theta({0, 0, 0, 0, 0, 0});
  for (size_t i=0; i<6; ++i) {
    theta[i] = theta[i] * pi / 180;
  }
  // tran = robot.forward_kinematics(theta);
  // std::cout << "M: \n" << tran.matrix() << std::endl;
  // std::cout << "q: " << Eigen::Quaterniond(tran.linear()) << std::endl;

  Eigen::Vector<double,6> axis1({1,0,0,0,0,1});
  Eigen::Vector<double,6> axis2({1,0,0,0,1,0});
  Eigen::Vector3d p(0,0,0), q(0,2,-2), c(1,1,0);
  // if (auto opt = pk_subproblem_1(axis2,q,c); !opt) {
  //   std::cout << "Error: no slove" << std::endl;
  // } else {
  //   std::cout << opt.value()*180/pi << std::endl;
  // }

  // std::vector<std::vector<double>> theta_2;
  // if (auto opt = pk_subproblem_2(axis2,axis1,p,q); !opt) {
  //   std::cout << "no solution" << std::endl;
  // } else {
  //   theta_2 = opt.value();
  // }
  // std::cout << "numSol = " << theta_2.size() << std::endl;
  // for (size_t i=0; i<theta_2.size(); ++i) {
  //   std::cout << "sol_2: " << theta_2[i][1]*180/pi << ", " << theta_2[i][0]*180/pi << std::endl;
  // }

  std::vector<double> theta_3;
  q = {0,2,1};
  if (auto opt = pk_subproblem_3(axis1,p,q, 1.2); !opt) {
    std::cout << "no solution" << std::endl;
  } else {
    theta_3 = opt.value();
  }
  for (size_t i=0; i<theta_3.size(); ++i) {
    std::cout << theta_3[i]*180/pi << std::endl;
  }

  // 求解逆运动学

  return 0;
}

