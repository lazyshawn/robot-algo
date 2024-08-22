#include "robot_prototype/robot_base.h"
#include "user_interface/user_interface.h"

const double pi = 3.1415926535897932384626433832795028841971693993751058209;

// std::string fname("data/config/fanuc_m20id25_scan.json");
// std::string fname("data/config/KUKA_KR210_R2700_EXTRA.json");
std::string fname("data/eston/config.json");
RobotBase robot;

void pk_problem();
void kinematics();
void calc();

int main(int argc, char** argv) {
  // 读取配置文件
  // if (!robot.load_config(fname)) {
  //   std::cout << "# Load robot config failed." << std::endl;
  //   return -1;
  // }
  // std::vector<double> theta = {4.4905, 5.5693, 21.4805, 0.2473, 65.8244, 129.9559};
  std::vector<double> theta = {-5.136, -7.4044, 34.2832, -0.2824, 65.9928, -43.9743};
  // std::vector<double> theta = {10, 10, 10, 10, 10, 10};
  // std::vector<double> theta = {10, 10, 10, 10, 10, 10};
  // std::vector<double> theta = {10, 10, 10, 10, 10, 10};

  Eigen::MatrixXd mat = get_random_matrix(2,100, 10);
  std::ofstream op("build/random.txt");
  for (size_t i=0; i<mat.cols(); ++i) {
    op << mat.col(i).transpose() << std::endl;
  }

  // kinematics();

  return 0;
}

Eigen::Matrix3d rpy_to_rot(const Eigen::Vector3d& euler) {
  Eigen::Matrix3d mat;

  double alpha = euler[2]*M_PI/180, beta = euler[1]*M_PI/180, gama = euler[0]*M_PI/180;

  mat(0,0) = cos(alpha)*cos(beta);
  mat(0,1) = cos(alpha)*sin(beta)*sin(gama) - sin(alpha)*cos(gama);
  mat(0,2) = cos(alpha)*sin(beta)*cos(gama) + sin(alpha)*sin(gama);

  mat(1,0) = sin(alpha)*cos(beta);
  mat(1,1) = sin(alpha)*sin(beta)*sin(gama) + cos(alpha)*cos(gama);
  mat(1,2) = sin(alpha)*sin(beta)*cos(gama) - cos(alpha)*sin(gama);

  mat(2,0) = -sin(beta);
  mat(2,1) = cos(beta)*sin(gama);
  mat(2,2) = cos(beta)*cos(gama);

  return mat;
}

Eigen::Vector3d rot_to_rpy(const Eigen::Matrix3d& mat) {
  Eigen::Vector3d euler;

  double ftemp = std::sqrt(mat(0,0) * mat(0,0) + mat(0,1) * mat(0,1));

  if (ftemp < 1e-6) {
    euler[0] = 0;
    euler[1] = atan2(-mat(0,2), ftemp);
    euler[2] = atan2(-mat(1,0), mat(1,1));
  } else {
    euler[0] = atan2(mat(1,2), mat(2,2));
    euler[1] = atan2(-mat(0,2), ftemp);
    euler[2] = atan2(mat(0,1), mat(0,0));
  }

  return euler;
}

void calc() {
  Eigen::Matrix3d final({{-sqrt(2)/2,0,sqrt(2)/2}, {0,1,0}, {-sqrt(2)/2,0,-sqrt(2)/2}});
  Eigen::Matrix3d pre({{sqrt(2)/2,0,sqrt(2)/2}, {0,-1,0}, {sqrt(2)/2,0,-sqrt(2)/2}});
  Eigen::AngleAxis rot(-M_PI/8, Eigen::Vector3d({1,0,0}));
  Eigen::Matrix3d aft = rot*pre;
  // std::cout.precision(18);
  std::cout << "Quaterniond = \n" << Eigen::Quaterniond(aft) << std::endl;

  Eigen::Isometry3d TargetFrame;
  TargetFrame.translation() = Eigen::Vector3d(1513.17, -112.956, -226.31);
  TargetFrame.linear() = Eigen::Matrix3d({{-0.70714, -4.78357e-05, 0.707071},
                                          {-3.38596e-06, 1, 3.37904e-05},
                                          {-0.707071, 0, -0.707143}});
  std::cout << TargetFrame.matrix() << std::endl;
}

void kinematics() {
  // 求解正运动学
  // 实际走的
  // std::vector<double> theta({17.1239, 71.083, -65.29274, 0, -24.7072, -17.1239});
  std::vector<double> theta({-25.0656, 2.85997, -43.6415, 125.922, 66.5106, -24.4094});
  // std::vector<double> theta({-5.719, 36.739, -40.95, 0, -49.053, -72.278});
  // std::vector<double> theta({-5.719, 36.739, -4.211, 0, -49.053, -72.278});
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


  // 求解逆运动学
  std::vector<std::vector<double>> ikSol;
  // tran.linear() = Eigen::Matrix3d({{0,0,1}, {0,1,0}, {-1,0,0}});
  // tran.translation() = Eigen::Vector3d(2274.1, 700.6, -282);
  Eigen::Isometry3d tran = robot.solve_forward_kinematics(theta);
  std::cout << "\nM: \n" << tran.matrix() << "\n" << std::endl;
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

