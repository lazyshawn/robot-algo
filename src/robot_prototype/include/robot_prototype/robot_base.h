#pragma once

#include "user_interface/user_interface.h"
#include "geometry/transform.h"

class robotBase {
public:
  // 机械臂别名
  std::string robotName;
  // 关节数、连杆数
  size_t numJoint, numLink;
  // 关节轴(twist)
  std::vector<Eigen::Vector<double,6>> jointAxis;
  // 关节角限位
  std::vector<double> jointLimit;
  // 执行器的零位位姿
  // Eigen::Isometry3d M0;
  std::vector<Eigen::Isometry3d> M0;
  // IP
  std::string ip;
  // 端口
  size_t port;
  bool j2j3Couple = false;

public:
  /* 
  * @brief : 从 json 文件中读取机械臂配置数据
  * @param : fname - 配置文件路径
  * @return: bool - 读取状态
  */
  bool load_config(std::string fname);
  /* 
  * @brief : 求解正运动学
  * @param : theta (rad) - 关节角
  * @param : eeIdx - 末端执行器序号
  * @return: 末端执行器位姿信息
  */
  Eigen::Isometry3d forward_kinematics(std::vector<double> theta, size_t eeIdx = 0) const;
  /* 
  * @brief : 肘形机械臂逆运动学 - Inverse kinematics of elbow manipulator
  * @param : theta (rad) - 关节角
  * @param : eeIdx - 末端执行器序号
  * @return: 末端执行器位姿信息
  */
  std::vector<std::vector<double>> inverse_kinematics(Eigen::Isometry3d tran, size_t eeIdx = 0) const;

  // virtual void read_robot_status(std::vector<double>& theta);
  // virtual void send_robot_cmd(const std::vector<double> theta);
};
