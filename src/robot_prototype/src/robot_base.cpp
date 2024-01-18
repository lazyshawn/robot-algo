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
  std::vector<std::string> requiredField({"end-effector", "PoE_joint"});
  for (auto field : requiredField) {
    if (!config.contains(field)) {
      printf("Error! # loadjson(): no required field \"%s\". ", field.c_str());
      return false;
    }
  }

  // 读取机械臂型号，非必要
  robotName = config["robot_brand"].get<std::string>();

  // 激活的关节数
  numJoint = config["num_of_joint"].get<size_t>();

  // 关节 2, 3 耦合: 关节 2 的运动会增加到关节 3
  jointCouple = config["j2_j3_coupling"].get<bool>();

  // 关节限位信息
  std::vector<double> upperJoint = config["joint_limits"]["positions"]["uppers"].get<std::vector<double>>();
  std::vector<double> lowerJoint = config["joint_limits"]["positions"]["lowers"].get<std::vector<double>>();
  jointLimit = std::vector<std::vector<double>>(numJoint);
  // 角度转弧度并保存
  for (size_t i=0; i<numJoint; ++i) {
    jointLimit[i] = {lowerJoint[i]*M_PI/180, upperJoint[i]*M_PI/180};
  }

  // * 读取 TCP 位姿
  Eigen::Isometry3d m0;
  m0.translation() = Eigen::Vector3d(config["end-effector"]["position"].get<std::vector<double>>().data());
  std::vector<double> xyzw = config["end-effector"]["quaternion"].get<std::vector<double>>();
  Eigen::Quaterniond tmpQua(xyzw[0], xyzw[1], xyzw[2], xyzw[3]);
  if (1 - tmpQua.norm() > 1e-9) {
    printf("Warning: # loadjson(): poor precision of quaternion.\n");
    tmpQua.normalize();
  }
  m0.linear() = tmpQua.matrix();
  M0.emplace_back(m0);

  // * 检查并获取关节数据
  nlohmann::json configJoint = config["PoE_joint"];
  // 关节数
  numJoint = configJoint.size();

  // 读取关节轴
  jointAxis = std::vector<Eigen::Vector<double,6>>(numJoint);
  for (size_t i=0; i<numJoint; ++i) {
    // 关节轴上任意点
    Eigen::Vector3d pnt(configJoint[i]["position"].get<std::vector<double>>().data());
    // 关节轴正方向，单位化
    Eigen::Vector3d dir(configJoint[i]["rotation_axis"].get<std::vector<double>>().data());
    dir.normalize();
    // 关节轴初始化
    jointAxis[i] << -dir.cross(pnt), dir;
  }

  Eigen::MatrixXd axis(numJoint, 6);
  for (size_t i=0; i<numJoint; ++i) {
    axis.row(i) = jointAxis[i];
  }

  isValid = true;
  return true;
}

Eigen::Isometry3d RobotBase::solve_forward_kinematics(std::vector<double> theta, size_t eeIdx) const {
  // 处理耦合情况
  if (jointCouple) {
    theta[2] += theta[1];
  }
  return forward_kinematics(jointAxis, theta, M0[eeIdx]);
}

std::optional<std::vector<std::vector<double>>>
RobotBase::solve_inverse_kinematics(const Eigen::Isometry3d &pose,
                                    const std::vector<double> &initJoint,
                                    size_t eeIdx) const {
  std::vector<std::vector<double>> solSet;
  // 获取所有不同构型的逆解
  if (auto opt = inverse_kinematics_elbow(jointAxis, pose, M0[eeIdx]); opt) {
    solSet = opt.value();
  } else {
    // 当解析解误差过大时使用数值逆解
  }
  if (!solSet.size()) {
    printf("Error: # solve_inverse_kinematics(): no IK solution exist.\n");
    return std::nullopt;
  }
  // 根据臂型过滤

  // 处理耦合情况: 理论关节角转为实际关节角
  if (jointCouple) {
    for (auto& jnt : solSet) {
      jnt[2] -= jnt[1];
    }
  }

  // 根据关节限位过滤不合理的构型
  std::vector<std::vector<double>> equSol;
  for (size_t i=0; i<solSet.size(); ++i) {
    // 获取等价的关节状态
    std::vector<std::vector<double>> tmp = get_equivalent_joint_state(solSet[i], jointLimit);
    equSol.insert(equSol.end(), tmp.begin(), tmp.end());
  }
  if (!equSol.size()) {
    printf("Error: # solve_inverse_kinematics(): no solution exist in jointLimit.\n");
    return std::nullopt;
  }
  return equSol;

  // 计算到零位最近的关节状态
  std::vector<double> preJoint = initJoint;
  if (preJoint.size() == 0) {
    preJoint = std::vector<double>(6, 0.0);
  }
  std::vector<double> sol = get_nearest_joint_state(equSol, preJoint);

  return std::vector<std::vector<double>>({sol});
}

std::vector<std::vector<double>> RobotBase::get_all_ik_solutions(const Eigen::Isometry3d &pose, size_t eeIdx) {
  std::vector<std::vector<double>> solSet;

  return solSet;
}
