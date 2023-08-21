#include "robot_prototype/robot_base.h"

bool RobotBase::load_config(std::string fname) {
  // * 读取配置文件
  nlohmann::json config;
  try {
    config = nlohmann::json::parse(std::ifstream(fname));
  } catch (...) {
    printf("Error! # load_json(): parse failed. ");
    return false;
  }

  // 检查需要的字段
  // std::vector<std::string> requiredField({"sdf", "sd", "sds"});
  // for (auto field : requiredField) {
  //   if (!config.contains(field)) {
  //     printf("Error! # loadjson(): no required field \"%s\". ", field.c_str());
  //     return false;
  //   }
  // }

  // 读取机械臂型号，非必要
  if (auto j = get_json_field(config, "robot_brand"); j) {
    robotName = j->get<std::string>();
  }

  // 激活的关节数
  numJoint = config["num_of_joint"].get<size_t>();

  // 关节 2, 3 耦合: 关节 2 的运动会增加到关节 3
  jointCouple = config["j2_j3_coupling"].get<bool>();

  // 关节限位信息
  std::vector<double> upperJoint = config["joint_limits"]["positions"]["uppers"].get<std::vector<double>>();
  std::vector<double> lowerJoint = config["joint_limits"]["positions"]["lowers"].get<std::vector<double>>();
  jointLimit = std::vector<std::vector<double>>(numJoint);
  for (size_t i=0; i<numJoint; ++i) {
    jointLimit[i] = {lowerJoint[i], upperJoint[i]};
  }

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

Eigen::Isometry3d RobotBase::solve_forward_kinematics(std::vector<double> theta, size_t eeIdx) const {
  // 处理耦合情况
  if (jointCouple) {theta[2] += theta[1];}
  return forward_kinematics(jointAxis, theta, M0[eeIdx]);
}

std::optional<std::vector<std::vector<double>>> RobotBase::solve_inverse_kinematics(Eigen::Isometry3d pose, size_t eeIdx) const {
  const double pi = 3.1415926535897932384626433832795028841971693993751058209;
  std::vector<std::vector<double>> ret;
  // 解析逆解
  if (auto opt = inverse_kinematics_elbow(jointAxis, pose, M0[eeIdx]); opt) {
    ret = opt.value();
  } else {
    // 当解析解误差过大时使用数值逆解
  }

  // 处理耦合情况
  if (jointCouple) {
    for (size_t i=0; i<ret.size(); ++i) {
      ret[i][2] -= ret[i][1];
    }
  }

  // 检验关节角是否超出限位区间
  for (size_t i=0; i<ret.size(); ++i) {
    wrap_joint(ret[i], jointLimit);
  }

  return ret;
}
