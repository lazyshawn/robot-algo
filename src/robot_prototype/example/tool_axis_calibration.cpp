#include "robot_prototype/robot_base.h"
#include "user_interface/user_interface.h"

std::string fname("data/config/YASKAWA_GP_280L.json");
RobotBase robot;

int main(int argc, char** argv) {
  // 读取配置文件
  if (!robot.load_config(fname)) {
    std::cout << "# Load robot config failed." << std::endl;
    return -1;
  }

  // 记录的关节角
  size_t num = 7;
  std::vector<std::vector<double>> joint(num);
  joint[0] = {-13.8214, 8.0335, -42.4806, -50.3095, -65.1551, -54.1884};
  joint[1] = {-13.3523, 9.1071, -42.3744, -23.2217, -87.4235, -25.1936};
  joint[2] = {-12.5203, 9.6335, -42.2645, 6.0095, -92.2848, 5.0641};
  joint[3] = {-11.5779, 8.6820, -42.4087, 42.2174, -74.6468, 42.9963};
  joint[4] = {-11.4089, 7.4800, -42.5442, 55.5469, -58.9059, 57.4414};
  joint[5] = {-12.0781, 6.4695, -42.6414, 72.1436, -28.3948, 78.7899};
  joint[6] = {-13.0171, 6.7354, -42.6294, 47.7293, -2.8863, 128.8763};
  for (auto& jnt : joint) {
    for (size_t i=0; i<jnt.size(); ++i) {
      jnt[i] *= M_PI/180;
    }
  }

  // 不同关节角下的 X 轴方向
  std::vector<Eigen::Vector3d> xDir(num);
  for (size_t i=0; i<num; ++i) {
    Eigen::Isometry3d tran = robot.solve_forward_kinematics(joint[i]);
    xDir[i] = tran.linear().col(0);
  }
  // 主轴方向
  Eigen::Vector3d center;
  double radius = circle_fitting(xDir, center);
  center.normalize();
  std::cout << "radius = " << radius << std::endl;
  std::cout.precision(18);
  std::cout << "center = " << center.transpose() << std::endl;

  // 最小二乘求零位时参考轴的朝向
  Eigen::MatrixXd A(3*num,3), b(3*num,1);
  for (size_t i=0; i<num; ++i) {
    Eigen::Isometry3d tran = robot.solve_forward_kinematics(joint[i]);
    A.block(3*i,0,3,3) = tran.linear();
    b.block(3*i,0,3,1) = center;
  }
  Eigen::Vector3d x0 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  std::cout << "X0 = " << x0.transpose() << std::endl;

  Eigen::Matrix3d pose;
  pose.col(0) = x0;
  pose.col(1) = (x0.cross(Eigen::Vector3d(1,0,0))).normalized();
  pose.col(2) = pose.col(0).cross(pose.col(1));
  std::cout << "pose =\n" << pose << std::endl;
  std::cout << "qua = " << Eigen::Quaterniond(pose) << std::endl;

  return 0;
}

