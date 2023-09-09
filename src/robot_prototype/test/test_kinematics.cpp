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
    std::vector<double> sol = robot.solve_inverse_kinematics(tran).value()[0];
    EXPECT_TRUE(tran.isApprox(robot.solve_forward_kinematics(sol)));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

