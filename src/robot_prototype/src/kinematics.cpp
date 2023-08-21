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
  typedef std::vector<std::vector<double>>::reverse_iterator vvRIte;

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
  solSet.reserve(8);

  // * e1 影响 pw 在 xoy 平面上的方向
  theta[0] = atan2(gPw[1], gPw[0]);
  solSet.push_back(theta);

  // * e3: pw, pb 之间的距离仅取决与 e3
  if (auto opt = pk_subproblem_3(jointAxis[2], pw, pb, (gPw-lieSE3(jointAxis[0]*theta[0])*pb).norm()); !opt) {
    printf("# inverse_kinematics_elbow(): q3.\n");
    return std::nullopt;
  } else {
    // 生成新的解
    for (vvRIte ite=solSet.rbegin(); ite!=solSet.rend(); ++ite) {
      (*ite)[2] = opt.value()[0];
      // 生成新的解
      theta = *ite;
      theta[2] = opt.value()[1];
      solSet.push_back(theta);
    }
  }

  // * e2: 每一组 e1,e3 有一个对应的 e2
  for (auto& sol : solSet) {
    if (auto opt = pk_subproblem_1(jointAxis[1], lieSE3(jointAxis[2]*sol[2])*pw, lieSE3(-jointAxis[0]*sol[0])*gPw); !opt) {
      printf("# inverse_kinematics_elbow(): q2.\n");
      return std::nullopt;
    } else {
      sol[1] = opt.value();
    }
  }

  // * e4, e5: 求解 e1,e2,e3 后 pe 的位置取决于 e4,e5
  for (vvRIte ite=solSet.rbegin(); ite!=solSet.rend(); ++ite) {
    // g2 = [ie3][ie2][ie1]g1 = [e4][e5][e6]
    Eigen::Isometry3d g2 = lieSE3(-jointAxis[2]*(*ite)[2])*lieSE3(-jointAxis[1]*(*ite)[1])*lieSE3(-jointAxis[0]*(*ite)[0])*g1;
    if (auto opt = pk_subproblem_2(jointAxis[3], jointAxis[4], pe, g2*pe); !opt) {
      printf("# inverse_kinematics_elbow(): q45.\n");
      return std::nullopt;
    } else {
      (*ite)[3] = opt.value()[0][0];
      (*ite)[4] = opt.value()[0][1];
      // 生成新的解
      theta = *ite;
      theta[3] = opt.value()[1][0];
      theta[4] = opt.value()[1][1];
      solSet.push_back(theta);
    }
  }

  // * e6: 针对每一组解计算对应的 e6
  for (vvRIte ite=solSet.rbegin(); ite!=solSet.rend(); ++ite) {
    Eigen::Isometry3d g2 = lieSE3(-jointAxis[2]*(*ite)[2])*lieSE3(-jointAxis[1]*(*ite)[1])*lieSE3(-jointAxis[0]*(*ite)[0])*g1;
    if (auto opt = pk_subproblem_1(jointAxis[5], pb, lieSE3(-jointAxis[4]*(*ite)[4])*lieSE3(-jointAxis[3]*(*ite)[3])*g2*pb); !opt) {
      printf("inverse_kinematics_elbow(): q6.\n");
      return std::nullopt;
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

