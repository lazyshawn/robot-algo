#include "robot_prototype/kinematics.h"

Eigen::Isometry3d serial_transfrom(const std::vector<Eigen::Vector<double, 6>>& jointAxis, const std::vector<double>& theta, size_t begIdx, size_t endIdx) {
  // 将默认终止序号设为最后一个关节轴序号
  endIdx = endIdx==-1 ? jointAxis.size()-1 : endIdx;

  Eigen::Isometry3d tran = Eigen::Isometry3d::Identity();
  // 计算空间正运动学
  size_t num = endIdx - begIdx + 1;
  for (size_t i=0; i<num; ++i) {
    int idx = endIdx - i;
    tran = lieSE3(jointAxis[idx] * theta[idx]) * tran;
  }

  return tran;
}

Eigen::Isometry3d forward_kinematics(const std::vector<Eigen::Vector<double,6>>& jointAxis, const std::vector<double>& theta, Eigen::Isometry3d M0) {
  return serial_transfrom(jointAxis, theta)*M0;
}

std::optional<std::vector<std::vector<double>>>
inverse_kinematics_elbow(const std::vector<Eigen::Vector<double,6>>& jointAxis, Eigen::Isometry3d pose, Eigen::Isometry3d M0) {

  // 解向量的迭代器
  typedef std::vector<std::vector<double>>::iterator vvRIte;

  // 腕关节点
  Eigen::Vector3d pw = get_twist_intersection({jointAxis[3], jointAxis[4], jointAxis[5]});
  // 肩关节点 (在关节2上一点)
  Eigen::Vector3d pb = get_point_on_twist(jointAxis[1]);
  // 1~6 关节旋转产生的运动
  Eigen::Isometry3d g1 = pose*M0.inverse();
  // pw 位置
  Eigen::Vector3d gPw = g1*pw;
  // 在 e6 上但不在 e4, e5 上的任意点
  Eigen::Vector3d pe = get_point_on_twist(jointAxis[5]) + jointAxis[5].tail(3)*pw.norm();

  // 记录一组解
  std::vector<double> theta(6,0);
  // 所有解的集合
  std::vector<std::vector<double>> solSet;
  // 临时存放多解的集合
  std::vector<std::vector<double>> tmpSet;
  solSet.reserve(8);

  // * e1 影响 pw 在 xoy 平面上的方向
  double dy = gPw[1], dx = gPw[0];
  for (size_t i=0; i<2; ++i) {
    theta[0] = atan2(dy, dx);
    // 根据关节 1 的正方向修正旋转角度
    if (jointAxis[0][5] < 0) theta[0] *= -1;
    solSet.push_back(theta);
    // 理论上需要考虑折叠的情况，故有八组解
    dy *= -1; dx *= -1;
  }

  // * e3: pw, pb 之间的距离仅取决与 e3
  tmpSet.clear();
  for (vvRIte ite=solSet.begin(); ite!=solSet.end(); ++ite) {
    if (auto opt = pk_subproblem_3(jointAxis[2], pw, pb, (gPw-lieSE3(jointAxis[0]*(*ite)[0])*pb).norm()); !opt) {
      printf("# inverse_kinematics_elbow(): q3, Loss 4 sols.\n");
      // 舍弃当前解
      solSet.erase(ite--);
    } else {
      (*ite)[2] = opt.value()[0];
      if (opt.value().size() == 1) {
        printf("# inverse_kinematics_elbow() : q3, equivalent sol, Loss 1 sol.\n");
        continue;
      }
      // 生成新的解
      theta = *ite;
      theta[2] = opt.value()[1];
      tmpSet.push_back(theta);
    }
  }
  solSet.insert(solSet.end(), tmpSet.begin(), tmpSet.end());

  // * e2: 每一组 e1,e3 有一个对应的 e2
  for (vvRIte ite=solSet.begin(); ite!=solSet.end(); ++ite) {
    if (auto opt = pk_subproblem_1(jointAxis[1], lieSE3(jointAxis[2]*(*ite)[2])*pw, lieSE3(-jointAxis[0]*(*ite)[0])*gPw); !opt) {
      printf("# inverse_kinematics_elbow(): q2, Loss 2 sols.\n");
      // 舍弃当前解，用反向迭代器删除元素
      solSet.erase(ite--);
    } else {
      (*ite)[1] = opt.value();
    }
  }

  // * e4, e5: 求解 e1,e2,e3 后 pe 的位置取决于 e4,e5
  tmpSet.clear();
  for (vvRIte ite=solSet.begin(); ite!=solSet.end(); ++ite) {
    // g2 = [ie3][ie2][ie1]g1 = [e4][e5][e6]
    Eigen::Isometry3d g2 = lieSE3(-jointAxis[2]*(*ite)[2])*lieSE3(-jointAxis[1]*(*ite)[1])*lieSE3(-jointAxis[0]*(*ite)[0])*g1;
    if (auto opt = pk_subproblem_2(jointAxis[3], jointAxis[4], pe, g2*pe); !opt) {
      printf("# inverse_kinematics_elbow(): q45, Loss 2 sols.\n");
      // 舍弃当前解
      solSet.erase(ite--);
    } else {
      (*ite)[3] = opt.value()[0][0];
      (*ite)[4] = opt.value()[0][1];
      if (opt.value().size() == 1) {
        printf("# inverse_kinematics_elbow() : q45, equivalent sol, Loss 1 sol.\n");
        continue;
      }
      // 生成新的解
      theta = *ite;
      theta[3] = opt.value()[1][0];
      theta[4] = opt.value()[1][1];
      tmpSet.push_back(theta);
    }
  }
  solSet.insert(solSet.end(), tmpSet.begin(), tmpSet.end());

  // * e6: 针对每一组解计算对应的 e6
  for (vvRIte ite=solSet.begin(); ite!=solSet.end(); ++ite) {
    Eigen::Isometry3d g2 = lieSE3(-jointAxis[2]*(*ite)[2])*lieSE3(-jointAxis[1]*(*ite)[1])*lieSE3(-jointAxis[0]*(*ite)[0])*g1;
    if (auto opt = pk_subproblem_1(jointAxis[5], pb, lieSE3(-jointAxis[4]*(*ite)[4])*lieSE3(-jointAxis[3]*(*ite)[3])*g2*pb); !opt) {
      printf("inverse_kinematics_elbow(): q6.\n");
      // 舍弃当前解
      solSet.erase(ite--);
    } else {
      (*ite)[5] = opt.value();
    }
  }

  return solSet;
}

bool wrap_joint(std::vector<double>& joint, const std::vector<std::vector<double>>& interval) {
  size_t num = joint.size();
  bool success = true;
  std::vector<double> theta(num,0);

  for (size_t i=0; i<joint.size(); ++i) {
    theta[i] = joint[i];
    // 当前关节角超出范围
    if (joint[i] < interval[i][0]*M_PI/180 || joint[i] > interval[i][1]*M_PI/180) {
      theta[i] = joint[i] + 2*M_PI;
      // 仍然超出范围
      if (theta[i] < interval[i][0]*M_PI/180 || theta[i] > interval[i][1]*M_PI/180) {
        theta[i] = joint[i] - 2*M_PI;
        if (theta[i] < interval[i][0]*M_PI/180 || theta[i] > interval[i][1]*M_PI/180) {
          success = false;
          printf("Error! # wrap_joint(): joint (%ld).\n", i+1);
          break;
        }
      }
      joint[i] = theta[i];
    }
  }
  return success;
}

std::vector<std::vector<double>> get_equivalent_joint_state(const std::vector<double>& jointState, const std::vector<std::vector<double>>& interval) {
  // 关节个数
  size_t num = jointState.size();
  // 返回的等价解
  std::vector<std::vector<double>> ret;

  // 每一个关节上处在限位区间内的等价角度值
  std::vector<std::vector<double>> option(num);
  for (size_t i=0; i<num; ++i) {
    // 检测关节角是否属于 [-2*PI, 2*PI]
    if (jointState[i] < -2*M_PI || jointState[i] > 2*M_PI) {
      printf("Error: #find_equivalent_joint_state() : theta[%zd] overrange. ", i);
      return ret;
    }
    double tmp = jointState[i];
    tmp += (tmp > 0) ? -2*M_PI : 2*M_PI;
    // 当前关节上的等价角度值
    std::vector<double> opt;
    // 当前关节角是否在限位区间内
    if (jointState[i] >= interval[i][0] && jointState[i] <= interval[i][1]) {
      opt.emplace_back(jointState[i]);
      // printf("jnt[%zd]: %f < %f > %f\n", i, interval[i][0], jointStack[i], interval[i][1]);
    }
    // 优弧角是否在限位区间内
    if (tmp >= interval[i][0] && tmp <= interval[i][1]) {
      opt.emplace_back(tmp);
      // printf("tmp[%zd]: %f < %f > %f\n", i, interval[i][0], tmp, interval[i][1]);
    }
    // 如果当前关节角无法转换到关节限位区间内，则等价解不存在
    if (!opt.size()) {
      return ret;
    }
    option[i] = opt;
  }

  // create combinations of several vectors without hardcoding loops
  // Ref: https://stackoverflow.com/a/1703575
  typedef std::vector<double>::iterator vecIte;
  std::vector<vecIte> ite(num);
  for (size_t i=0; i<num; ++i) {
    ite[i] = option[i].begin();
  }
  while (ite[0] != option[0].end()) {
    // process the pointed-to elements
    std::vector<double> joint(num);
    for (size_t i=0; i<num; ++i) {
      joint[i] = *ite[i];
    }
    ret.push_back(joint);

    // the following increments the "odometer" by 1
    ++ite[num-1];
    for (int i = num-1; (i > 0) && (ite[i] == option[i].end()); --i) {
      ite[i] = option[i].begin();
      ++ite[i-1];
    }
  }

  return ret;
}

std::vector<double> get_nearest_joint_state(const std::vector<std::vector<double>>& jointStack, const std::vector<double>& goal) {
  if (!jointStack.size()) return std::vector<double>();
  size_t numJoint = jointStack[0].size();

  // 默认计算到零位的距离
  std::vector<double> goalJoint = goal;
  if (!goal.size()) {
    goalJoint = std::vector<double>(numJoint, 0.0);
  }

  // 计算每一组关节状态到目标位置需要转动的角度
  std::vector<double> manhatonDist(jointStack.size(), std::numeric_limits<double>::infinity());
  for (size_t i=0; i<jointStack.size(); ++i) {
    double tmp = 0.0;
    for (size_t j=0; j<numJoint; ++j) {
      tmp += std::fabs(jointStack[i][j] - goalJoint[j]);
    }
    manhatonDist[i] = tmp;
  }

  // 查找最近关节状态的序号
  size_t idx = 0;
  double minDist = std::numeric_limits<double>::infinity();
  for (size_t i=0; i<jointStack.size(); ++i) {
    if (manhatonDist[i] < minDist) {
      minDist = manhatonDist[i];
      idx = i;
    }
  }
  return jointStack[idx];
}

bool satisfy_joint_limit(const std::vector<double>& joint, const std::vector<std::vector<double>>& interval) {
  for (size_t i=0; i<joint.size(); ++i) {
    if (joint[i] < interval[i][0] || joint[i] > interval[i][1]) {
      return false;
    }
  }
  return true;
}
