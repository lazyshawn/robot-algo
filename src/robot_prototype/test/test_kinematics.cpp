#include <gtest/gtest.h>

#include "robot_prototype/robot_base.h"
#include "user_interface/user_interface.h"

const double pi = 3.1415926535897932384626433832795028841971693993751058209;

// 测试关节角
const std::vector<std::vector<double>> testTheta{
  {17.1239, 71.083, -65.29274, 0, -24.7072, -17.1239},
  {-5.719, 136.739, -40.95, 0, -49.053, -72.278},
  {3.567,  48.680, -32.630, 0.047, -56.355, 152.630},
  {90, 48.406, -90, -0.021, -57.042, 157.207},
  {0, 0, 0, 0, 0, 0}
};

// std::string fname("data/config/fanuc_m20id25_scan.json");
// std::string fname("data/config/KUKA_KR210_R2700_EXTRA.json");
std::string fname("data/config/YASKAWA_GP_280L.json");
RobotBase robot;

TEST(Kinematics, load_config) {
  std::cout << std::filesystem::current_path() << std::endl;
  ASSERT_TRUE(robot.load_config(fname));
}

TEST(Kinematics, forward_kinematics) {
  robot.load_config(fname);
  for (auto theta : testTheta) {
    for (size_t i=0; i<6; ++i) {
      std::cout << theta[i] << ", ";
      theta[i] = theta[i] * pi / 180;
    }
    Eigen::Isometry3d tran = robot.solve_forward_kinematics(theta);
    std::cout << "\n" << tran.matrix() << "\n" << std::endl;
  }
}

TEST(Kinematics, inverse_kinematics) {
  robot.load_config(fname);
  Eigen::Isometry3d tran = Eigen::Isometry3d::Identity();
  for (auto theta : testTheta) {
    if (!satisfy_joint_limit(theta, robot.jointLimit))
      continue;
    tran = robot.solve_forward_kinematics(theta);
    if (auto opt = robot.solve_inverse_kinematics(tran); opt) {
      std::vector<double> sol = opt.value()[0];
      EXPECT_TRUE(tran.isApprox(robot.solve_forward_kinematics(sol)));
    } else {
    }
  }
}

TEST(algebra, circle_fitting) {
  size_t num = 10;
  // 随机误差
  Eigen::MatrixXd randMat = get_random_matrix(3, num, 0, 0.001);
  std::vector<double> randNum = get_uniform_double(num+2);

  double radius = randNum[num]*10;
  Eigen::Vector3d center(1,2,3), axis = get_random_matrix(3,1);
  axis.normalize();

  std::vector<Eigen::Vector3d> samples;
  for (size_t i=0; i<num; ++i) {
    double theta = randNum[i]*2*M_PI;
    // xoy 平面中特定圆上一个点
    Eigen::Vector3d pnt(sin(theta)*radius, cos(theta)*radius, 0);
    // 绕轴旋转 + 平移
    pnt = (Eigen::AngleAxisd(randNum[2]*2*M_PI, axis)*pnt).eval();
    pnt += center;
    // 随机误差
    pnt += randMat.col(i);
    samples.push_back(pnt);
  }
  Eigen::Vector3d fitCenter;
  double fitRadius = circle_fitting(samples, fitCenter);
  std::cout << "raw data:" << std::endl;
  std::cout << "r = " << radius << ", center = " << center.transpose() << std::endl;
  std::cout << "fit data:" << std::endl;
  std::cout << "r = " << fitRadius << ", center = " << fitCenter.transpose() << std::endl;
  ASSERT_TRUE((radius-fitRadius)/radius < 1e-2);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

