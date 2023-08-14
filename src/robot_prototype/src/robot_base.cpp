#include "robot_prototype/robot_base.h"

bool robotBase::load_config(std::string fname) {
  // * 读取配置文件
  nlohmann::json config = nlohmann::json::parse(std::ifstream(fname));

  // 读取机械臂型号，非必要
  if (auto j = get_json_field(config, "robot_brand"); j) {
    robotName = j->get<std::string>();
  }

  // 关节 2, 3 耦合
  j2j3Couple = config["j2_j3_coupling"].get<bool>();
  // * 读取 TCP 位姿
  if (auto cfg = get_json_field(config, "end-effector"); cfg) {
    Eigen::Isometry3d m0;
    m0.translation() = Eigen::Vector3d((*cfg)["position"].get<std::vector<double>>().data());
    std::vector<double> xyzw = (*cfg)["quaternion"].get<std::vector<double>>();
    m0.linear() = Eigen::Quaterniond(xyzw[0], xyzw[1], xyzw[2], xyzw[3]).matrix();
    M0.emplace_back(m0);
    // std::cout << "M0:\n" << m0.matrix() << std::endl;
  } else {
    std::cout << "Error: read [end-effector] failed." << std::endl;
    return false;
  }

  // * 检查并获取关节数据
  std::optional<nlohmann::json> configJoint = get_json_field(config, "PoE_joint");
  if(!configJoint) {
    std::cout << "error" << std::endl;
  }
  // 关节数
  numJoint = configJoint->size();

  // 读取关节轴
  jointAxis = std::vector<Eigen::Vector<double,6>>(numJoint);
  for (size_t i=0; i<numJoint; ++i) {
    // 关节轴上任意点
    Eigen::Vector3d pnt((*configJoint)[i]["position"].get<std::vector<double>>().data());
    // 关节轴正方向，单位化
    Eigen::Vector3d dir((*configJoint)[i]["rotation_axis"].get<std::vector<double>>().data());
    dir.normalize();
    // 关节轴初始化
    jointAxis[i] << -dir.cross(pnt), dir;
  }

  Eigen::MatrixXd axis(numJoint, 6);
  for (size_t i=0; i<numJoint; ++i) {
    axis.row(i) = jointAxis[i];
  }
  // std::cout << "Joint Axis:\n" << axis << std::endl;

  return true;
}

Eigen::Isometry3d robotBase::forward_kinematics(std::vector<double> theta, size_t eeIdx) const {
  // 处理耦合情况
  if (j2j3Couple) {theta[2] += theta[1];}
  Eigen::Isometry3d tran = M0[eeIdx];
  for (size_t i=0; i<numJoint; ++i) {
    int idx = numJoint - i - 1;
    tran = lieSE3(jointAxis[idx] * theta[idx]) * tran;
  }
  return tran;
}

std::vector<std::vector<double>> robotBase::inverse_kinematics(Eigen::Isometry3d tran, size_t eeIdx) const {
  // 肘关节点， 腕关节点
  Eigen::Vector3d pb, bw;

}
