/**
* @file   robot_base.h
* @brief  机械臂几何模型.
*
* 机械臂算法: 正逆运动学
*/
#pragma once

#include <nlohmann/json.hpp>
#include <fstream>

#include "robot_prototype/kinematics.h"

/**
* @brief  Robot prototype.
*
* 机械臂配置文件信息, 几何信息
*/
class RobotBase {
public:
  //! 机械臂别名
  std::string robotName;
  //! 关节数、连杆数
  size_t numJoint, numLink;
  //! 关节轴(twist)
  std::vector<Eigen::Vector<double,6>> jointAxis;
  //! 关节角限位
  std::vector<std::vector<double>> jointLimit;
  //! 执行器的零位位姿
  std::vector<Eigen::Isometry3d> M0;
  //! IP
  std::string ip;
  //! 端口
  size_t port;
  //! 关节耦合标志: 上游关节的运动会累加到下游关节
  bool jointCouple = false;
  //! 是否已经加载配置文件
  bool isValid = false;
  uint8_t configType;

public:
  /**
  * @brief  从 json 文件中读取机械臂配置数据
  * @param  [in] fname 配置文件路径
  * @return 0 - 读取成功; 1 - 读取失败;
  */
  bool load_config(std::string fname);
  /**
  * @brief  求解正运动学
  * @param  [in] theta 关节角 (rad)
  * @param  [in] eeIdx 末端执行器序号
  * @return 末端执行器位姿信息
  */
  Eigen::Isometry3d solve_forward_kinematics(std::vector<double> theta, size_t eeIdx = 0) const;
  /**
  * @brief  肘形机械臂逆运动学 - Inverse kinematics of elbow manipulator
  * @param  pose 齐次转换矩阵
  * @param  initJoint 初始关节角
  * @param  eeIdx 末端执行器的序号
  * @return 距离初始关节角最近的逆解
  */
  virtual std::optional<std::vector<std::vector<double>>> solve_inverse_kinematics(const Eigen::Isometry3d &pose, const std::vector<double> &initJoint = {}, size_t eeIdx = 0) const;

  /**
  * @brief  肘形机械臂的所有逆运动学解
  * @param  pose 齐次转换矩阵
  * @param  eeIdx 末端执行器的序号
  * @return 所有合理的逆运动学解
  */
  std::vector<std::vector<double>> get_all_ik_solutions(const Eigen::Isometry3d &pose, size_t eeIdx = 0);

  // 1. 距离初始关节角最近的解
  // 1. 满足特定构型的解 elbow_on, wrist_inverse, shoulder_right, etc.

  // 驱动中添加机械臂控制程序
  // virtual void read_robot_status(std::vector<double>& theta);
  // virtual void send_robot_cmd(const std::vector<double> theta);
};
