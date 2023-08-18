#include "robot_prototype/robot_base.h"
#include "user_interface/user_interface.h"

const double pi = 3.1415926535897932384626433832795028841971693993751058209;

std::string fname("data/config/fanuc_m20id25_scan.json");
RobotBase robot;

void pk_problem();
void kinematics();
void calc();

int main(int argc, char** argv) {
  // 读取配置文件
  if (!robot.load_config(fname)) {
    std::cout << "# Load robot config failed." << std::endl;
    return -1;
  }

  kinematics();

  return 0;
}

void calc() {
  // double theta = -pi/8;
  // Eigen::Quaterniond pre(0.717991389577189, -0.581413977297589, 0.297399432761697, -0.240831328711687);
  // Eigen::Quaterniond aft(0.18024, 0.906127, 0.0746578, 0.37533);
  // std::cout << aft*pre.inverse() << std::endl;

  Eigen::Isometry3d TargetFrame;
  TargetFrame.translation() = Eigen::Vector3d(1513.17, -112.956, -226.31);
  TargetFrame.linear() = Eigen::Matrix3d({{-0.70714, -4.78357e-05, 0.707071},
                                          {-3.38596e-06, 1, 3.37904e-05},
                                          {-0.707071, 0, -0.707143}});
  std::cout << TargetFrame.matrix() << std::endl;

  // std::cout << "pre = " << pre << std::endl;
  // for (size_t i=0; i<9; ++i) {
  //   double q = -pi/8/2 * (i+1);
  //   Eigen::AngleAxis axis(q, Eigen::Vector3d(1,0,0));
  //   std::cout << "Rot(x,-pi/" << 8*2/(i+1) << ") : " << axis*pre << std::endl;
  // }
}

void kinematics() {
  // 求解正运动学
  // std::vector<double> theta({0,-M_PI/2, 0,0,M_PI/2,0});
  Eigen::Matrix3d final({{-sqrt(2)/2,0,sqrt(2)/2}, {0,1,0}, {-sqrt(2)/2,0,-sqrt(2)/2}});
  Eigen::Matrix3d pre({{sqrt(2)/2,0,sqrt(2)/2}, {0,-1,0}, {sqrt(2)/2,0,-sqrt(2)/2}});
  Eigen::AngleAxis rot(-pi/8, Eigen::Vector3d({1,0,0}));
  Eigen::Matrix3d aft = rot*pre;
  // std::cout.precision(18);
  std::cout << "Quaterniond = \n" << Eigen::Quaterniond(aft) << std::endl;
  // 实际走的
  std::vector<double> theta({-5.719, 36.739, -40.95, 0, -49.053, -72.278});
  // std::vector<double> theta({-2.198, 36.871, -40.791, 0, -49.212, -73.303});
  // 应该走的
  // std::vector<double> theta({3.567,  48.680, -32.630, 0.047, -56.355, 152.630});
  // std::vector<double> theta({90, 48.406, -90, -0.021, -57.042, 157.207});
  // std::vector<double> theta({0, 0, 0, 0, 0, 0});
  for (size_t i=0; i<6; ++i) {
    std::cout << theta[i] << ", ";
    theta[i] = theta[i] * pi / 180;
  }
  std::cout << std::endl;
  Eigen::Isometry3d tran = robot.solve_forward_kinematics(theta);
  std::cout << "\nM: \n" << tran.matrix() << std::endl;
  // std::cout << "q: " << Eigen::Quaterniond(tran.linear()) << std::endl;
  // Eigen::Matrix3d rotmat = tran.linear()*(robot.M0[0].linear().inverse());
  // std::cout << "rotmat:\n" << rotmat << std::endl;
  // std::cout << "\nM0 should be:\n" << rotmat.inverse()*final << std::endl;


  // 求解逆运动学
  std::vector<std::vector<double>> ikSol;
  if (auto opt = robot.solve_inverse_kinematics(tran); !opt) {
    std::cout << "No inverse kinematics solution." << std::endl;
    return;
  } else {
    ikSol = opt.value();
  }

  for (size_t i=0; i<ikSol.size(); ++i) {
    for (size_t j=0; j<6; ++j) {
      std::cout << ikSol[i][j]*180/pi << ", ";
    }
    std::cout << std::endl;
    std::cout << robot.solve_forward_kinematics(ikSol[i]).matrix() << "\n" << std::endl;
  }

}

void pk_problem() {
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

  // std::vector<double> theta_3;
  // axis1 = {840, 0, -75, 0, -1, 0};
  // p = {965, 0, 1055}, q = {75, 0, 0};
  // double distance = 1513;
  // if (auto opt = pk_subproblem_3(axis1,p,q, distance); !opt) {
  //   std::cout << "no solution" << std::endl;
  // } else {
  //   theta_3 = opt.value();
  // }
  // for (size_t i=0; i<theta_3.size(); ++i) {
  //   std::cout << theta_3[i]*180/pi << std::endl;
  //   std::cout << "d = " << distance << ", get: " << (lieSE3(axis1*theta_3[i])*p - q).norm() << std::endl;
  // }
}

