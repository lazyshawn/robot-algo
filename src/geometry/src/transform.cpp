#include "geometry/transform.h"
// #include <iostream>

Eigen::Matrix3d euler2SO3(std::vector<double> theta, std::vector<size_t> axisIdx) {
  const std::vector<Eigen::Vector3d> axis = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};

  Eigen::Matrix3d rotmat = Eigen::Matrix3d::Identity();
  rotmat = Eigen::AngleAxisd(theta[2], axis[axisIdx[2]])
    * Eigen::AngleAxisd(theta[1], axis[axisIdx[1]])
    * Eigen::AngleAxisd(theta[0], axis[axisIdx[0]]);

  return rotmat;
}

Eigen::Matrix4d liese3(Eigen::Vector<double,6> twist) {
  Eigen::Vector3d v = twist.head<3>(), w = twist.tail<3>();

  Eigen::Matrix4d exponent = Eigen::Matrix<double,4,4>::Zero();
  exponent.topLeftCorner(3,3) = lieso3(w);
  exponent.topRightCorner(3,1) = v;

  return exponent;
}

Eigen::Matrix3d lieso3(Eigen::Vector<double,3> vec) {
  // 反对称矩阵
  return Eigen::Matrix3d({{0, -vec[2], vec[1]}, {vec[2], 0, -vec[0]}, {-vec[1], vec[0], 0}});
}

Eigen::Matrix3d lieSO3(Eigen::Vector<double,3> vec) {
  return lieso3(vec).exp();
}

Eigen::Isometry3d lieSE3(Eigen::Vector<double,6> twist) {
  // 矩阵指数
  Eigen::Matrix4d mat = liese3(twist).exp();
  return Eigen::Isometry3d(mat);
}

std::optional<Eigen::Vector3d> so3toAxis(Eigen::Matrix3d soMat) {
  // 判断是否为反对称矩阵
  // Ref: https://stackoverflow.com/a/50783317
  Eigen::Matrix3d skewSym = -soMat.transpose();
  if (!soMat.isApprox(skewSym)) {
    printf("Warnning: skew symmetrix matrix check failed.\n");
  }
  return std::make_optional<Eigen::Vector3d>(Eigen::Vector3d({soMat(2,1), soMat(0,2), soMat(1,0)}));
}

std::optional<Eigen::Vector<double,6>> SE3toTwist(Eigen::Isometry3d tran) {
  // 求矩阵对数
  Eigen::Matrix4d se3 = tran.matrix().log();
  // 旋转轴
  Eigen::Vector3d w = so3toAxis(se3.topLeftCorner(3,3)).value();
  double theta = w.norm();
  w /= theta;
  // 空间速度
  Eigen::Vector3d v = se3.topRightCorner(3,1) / theta;

  Eigen::Vector<double,6> twist;
  twist << v, w;
  return std::make_optional<Eigen::Vector<double,6>>(twist);
}

Eigen::Vector3d get_point_on_twist(Eigen::Vector<double,6> twist) {
  Eigen::Vector3d v = twist.head<3>(), w = twist.tail<3>();
  double squareTheta = w.squaredNorm();
  return squareTheta == 0 ? Eigen::Vector3d({0,0,0}) : w.cross(v) / squareTheta;
}

Eigen::Vector3d get_twist_intersection(std::vector<Eigen::Vector<double,6>> twist) {
  const int numTwist = twist.size();
  Eigen::MatrixXd A(2*numTwist,3);
  Eigen::VectorXd b(2*numTwist);
  for (size_t i=0; i<numTwist; ++i) {
    // 螺旋轴方向向量
    Eigen::Vector3d w = twist[i].tail(3);
    // 获取补子空间基底
    Eigen::Matrix3d basis = construct_unit_orthogonal_basis(w);
    // 两个包含螺旋轴的平面的法向量
    Eigen::Vector3d w1(basis(0,1), basis(1,1), basis(2,1)), w2(basis(0,2), basis(1,2), basis(2,2));
    // 螺旋轴上的任意点
    Eigen::Vector3d ri = get_point_on_twist(twist[i]);
    // 构造最小二乘矩阵
    A.row(2*i) = w1.transpose();
    A.row(2*i+1) = w2.transpose();
    b[2*i] = w1.dot(ri);
    b[2*i+1] = w2.dot(ri);
  }
  // 两螺旋轴的交点 r
  // Eigen::Vector3d r = (A.transpose()*A).inverse()*A.transpose()*b;
  return (A.transpose()*A).inverse()*A.transpose()*b;
}

std::optional<double> pk_subproblem_1(Eigen::Vector<double,6> twist, Eigen::Vector3d p, Eigen::Vector3d q) {
  // 螺旋轴上一点 r
  Eigen::Vector3d r = get_point_on_twist(twist);
  // 螺旋轴方向向量
  Eigen::Vector3d w = twist.tail(3);
  w.normalize();
  // 从点 r 到点 p, q 的向量
  Eigen::Vector3d u = p-r, v = q-r;
  // u, v 在垂直于螺旋轴的平面上的投影
  Eigen::Vector3d up = u - w*w.transpose()*u, vp = v - w*w.transpose()*v;

  // 解的存在性条件
  if (std::fabs(w.dot(u) - w.dot(v)) > 1e-9 || std::fabs(up.norm() - vp.norm()) > 1e-9) {
    printf("Error! # pk_subproblem_1(): %f, %f. ", std::fabs(w.dot(u) - w.dot(v)), std::fabs(up.squaredNorm() - vp.squaredNorm()));
    return std::nullopt;
  }
  return atan2(w.dot(up.cross(vp)),up.dot(vp));
}

std::optional<std::vector<std::vector<double>>> pk_subproblem_2(Eigen::Vector<double,6> twist1, Eigen::Vector<double,6> twist2, Eigen::Vector3d p, Eigen::Vector3d q) {
  std::vector<Eigen::Vector<double,6>> twist({twist1, twist2});
  std::vector<Eigen::Vector3d> w({twist1.tail(3), twist2.tail(3)});
  // 两螺旋轴的交点 r
  Eigen::Vector3d r = get_twist_intersection({twist1,twist2});

  // * 求解 pk 问题
  // 从点 r 到点 p, q 的向量
  Eigen::Vector3d u = p - r, v = q - r;
  double w12 = w[0].dot(w[1]);
  double alpha = (w12*w[1].dot(u) - w[0].dot(v)) / (w12*w12 - 1), beta = (w12*w[0].dot(v) - w[1].dot(u)) / (w12*w12 - 1);
  double squaredGama = (u.squaredNorm() - alpha*alpha - beta*beta - 2*alpha*beta*w12) / (w[0].cross(w[1]).squaredNorm());
  // 解的存在性条件
  size_t numSol = 0;
  if (squaredGama < 0) {
    printf("Error! # pk_subproblem_2(): gama < 0. ");
    return std::nullopt;
  } else if (squaredGama < 1e-9) {
    // 一组解
    numSol = 1;
  } else {
    // 两组解
    numSol = 2;
  }

  // 始终返回两组解
  double gama = sqrt(squaredGama);
  std::vector<Eigen::Vector3d> z(2);
  z[0] = alpha*w[0] + beta*w[1] + gama*(w[0].cross(w[1]));
  z[1] = z[0] - 2*gama*(w[0].cross(w[1]));

  std::vector<std::vector<double>> ret;
  for (size_t i=0; i<numSol; ++i) {
    std::vector<double> theta(2,0);
    // std::cout << "c = " << (z[i] + r).transpose() << std::endl;
    if (auto opt = pk_subproblem_1(twist[1], p, z[i] + r); !opt) {
      printf("# pk_subproblem_2(): no pk_1, p -> c. ");
      return std::nullopt;
    } else {
      theta[1] = opt.value();
    }
    if (auto opt = pk_subproblem_1(twist[0], q, z[i] + r); !opt) {
      printf("# pk_subproblem_2(): no pk_1, p -> c. ");
      return std::nullopt;
    } else {
      theta[0] = -1 * opt.value();
    }
    ret.emplace_back(theta);
  }

  return ret;
}

std::optional<std::vector<double>> pk_subproblem_3(Eigen::Vector<double,6> twist, Eigen::Vector3d p, Eigen::Vector3d q, double distance) {
  // 螺旋轴上的任意点
  Eigen::Vector3d r = get_point_on_twist(twist);
  // 螺旋轴方向向量
  Eigen::Vector3d w = twist.tail(3);
  // 从点 r 到点 p, q 的向量
  Eigen::Vector3d u = p - r, v = q - r;
  // u, v 在垂直于螺旋轴的平面上的投影
  Eigen::Vector3d up = u - w*w.transpose()*u, vp = v - w*w.transpose()*v;
  double squaredDp = distance*distance - (w.dot(p-q))*(w.dot(p-q));
  double q0 = atan2(w.dot(up.cross(vp)), up.dot(vp));
  std::vector<double> dq(2);
  dq[0] = acos((up.squaredNorm() + vp.squaredNorm() - squaredDp) / (2*up.norm()*vp.norm()));
  dq[1] = -dq[0];

  size_t numSol = 0;
  std::vector<double> ret;
  double minDis = std::fabs(up.norm() - vp.norm()), maxDis = up.norm() + vp.norm();
  // 解的存在性条件: |up-vp| <= dp <= up+vp
  if (std::sqrt(squaredDp) < minDis || std::sqrt(squaredDp) > maxDis) {
    return std::nullopt;
  } else if (std::fabs(std::sqrt(squaredDp) - minDis) < 1e-9 || std::fabs(std::sqrt(squaredDp) - maxDis) < 1e-9) {
    numSol = 1;
  } else {
    numSol = 2;
  }

  for (size_t i=0; i<numSol; ++i) {
    ret.emplace_back(q0+dq[i]);
  }
  return ret;
}

