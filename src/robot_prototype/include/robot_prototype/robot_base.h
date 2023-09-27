#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "robot_prototype/kinematics.h"

class RobotBase {
public:
  // 机械臂别名
  std::string robotName;
  // 关节数、连杆数
  size_t numJoint, numLink;
  // 关节轴(twist)
  std::vector<Eigen::Vector<double,6>> jointAxis;
  // 关节角限位
  std::vector<std::vector<double>> jointLimit;
  // 执行器的零位位姿
  std::vector<Eigen::Isometry3d> M0;
  // IP
  std::string ip;
  // 端口
  size_t port;
  // 关节耦合标志: 上游关节的运动会累加到下游关节
  bool jointCouple = false;

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
  Eigen::Isometry3d solve_forward_kinematics(std::vector<double> theta, size_t eeIdx = 0) const;
  /* 
  * @brief : 肘形机械臂逆运动学 - Inverse kinematics of elbow manipulator
  * @param : theta (rad) - 关节角
  * @param : eeIdx - 末端执行器序号
  * @return: 末端执行器位姿信息
  */
  virtual std::optional<std::vector<std::vector<double>>> solve_inverse_kinematics(const Eigen::Isometry3d &pose, const std::vector<double> &initJoint = {}, size_t eeIdx = 0) const;

  // 驱动中添加机械臂控制程序
  // virtual void read_robot_status(std::vector<double>& theta);
  // virtual void send_robot_cmd(const std::vector<double> theta);
};
