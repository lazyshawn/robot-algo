#include "geometry/transform.h"
#include <iostream>
#include <complex>

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
  if (std::fabs(w.dot(u) - w.dot(v)) > 1e-7 || std::fabs(up.norm() - vp.norm()) > 1e-7) {
    printf("Error! # pk_subproblem_1(): %.9f, %.9f. ", std::fabs(w.dot(u) - w.dot(v)), std::fabs(up.norm() - vp.norm()));
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
  } else if (std::sqrt(squaredGama) < 1e-9) {
    // 一组解
    numSol = 1;
  } else {
    // 两组解
    numSol = 2;
  }

  // 始终返回两组解
  double gama = std::sqrt(squaredGama);
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
    printf("Error: # pk_subproblem_3(): %.9f < %.9f > %.9f. ", minDis, std::sqrt(squaredDp), maxDis);
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

double canonical_subproblem_1(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2) {
  k.normalize();
  Eigen::Matrix3d kx = lieso3(k);

  return std::atan2((kx*p1).dot(p2), -(kx*kx*p1).dot(p2));
}

bool verify_canonical_subproblem_1(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double q) {
  k.normalize();
  double ans = (lieSO3(k*q)*p1 - p2).norm();
  printf("Info: #check_canonical_subproblem_1(): delta = %.10f\n", ans);
  return (std::fabs(ans) < 1e-9);
}

std::vector<std::vector<double>> canonical_subproblem_2(Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d p2, Eigen::Vector3d k2) {
  k1.normalize();
  k2.normalize();

  std::vector<std::vector<double>> ans;
  std::vector<double> q1 = canonical_subproblem_4(p1, k1, k2, k2.dot(p2));
  for (auto& q : q1) {
    double q2 = canonical_subproblem_1(p2, k2, lieSO3(k1*q)*p1);
    ans.push_back({q, q2});
  }

  return ans;
}

bool verify_canonical_subproblem_2(Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d p2, Eigen::Vector3d k2, double q1, double q2) {
  double ans = (lieSO3(k1*q1)*p1 - lieSO3(k2*q2)*p2).norm();
  printf("Info: #check_canonical_subproblem_2(): delta = %.10f\n", ans);
  return (std::fabs(ans) < 1e-9);
}

std::vector<double> canonical_subproblem_3(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double d) {
  double dp = (p1.squaredNorm() + p2.squaredNorm() - d*d)/2/p2.norm();
  return canonical_subproblem_4(p1, k, p2.normalized(), dp);
}

bool verify_canonical_subproblem_3(Eigen::Vector3d p1, Eigen::Vector3d k, Eigen::Vector3d p2, double d, double q) {
  double ans = (lieSO3(k*q)*p1 - p2).norm() - d;
  printf("Info: #check_canonical_subproblem_3(): delta = %.10f\n", ans);
  return (std::fabs(ans) < 1e-9);
}

std::vector<double> canonical_subproblem_4(Eigen::Vector3d p, Eigen::Vector3d k, Eigen::Vector3d h, double d) {
  Eigen::Matrix3d kx = lieso3(k);
  Eigen::MatrixXd Akp(3,2);
  Akp << kx*p, -kx*kx*p;

  Eigen::Matrix<double,1,2> A = h.transpose()*Akp;
  double b = d - h.transpose()*k*k.transpose()*p;

  Eigen::Vector2d xmin = A.transpose()*(A*A.transpose()).inverse()*b;
  std::vector<Eigen::Vector2d> x;
  x.reserve(2);
  // 两解
  if (xmin.squaredNorm() < 1 - 1e-12) {
    // Eigen::Vector2d xNp = Eigen::Matrix2d({{0,1}, {-1,0}}) * A.transpose();
    Eigen::Vector2d xNp(A(1), -A(0));
    double coord = sqrt(1-xmin.squaredNorm()) / xNp.norm();
    x.push_back(xmin + xNp*coord);
    x.push_back(xmin - xNp*coord);
  } else {
    // 一解
    x.push_back(xmin);
  }

  std::vector<double> ans;
  for (size_t i=0; i<x.size(); ++i) {
    ans.push_back(std::atan2(x[i](0), x[i](1)));
  }
  return ans;
}

bool verify_canonical_subproblem_4(Eigen::Vector3d p, Eigen::Vector3d k, Eigen::Vector3d h, double d, double q) {
  h.normalize();
  k.normalize();
  double ans = h.transpose()*lieSO3(k*q)*p;
  printf("Info: #check_canonical_subproblem_4(): delta = %.10f\n", ans - d);
  return (std::fabs(ans - d) < 1e-9);
}

std::vector<std::vector<double>> canonical_subproblem_6(
    Eigen::Vector3d p1, Eigen::Vector3d k1, Eigen::Vector3d h1,
    Eigen::Vector3d p2, Eigen::Vector3d k2, Eigen::Vector3d h2, double d1,
    Eigen::Vector3d p3, Eigen::Vector3d k3, Eigen::Vector3d h3,
    Eigen::Vector3d p4, Eigen::Vector3d k4, Eigen::Vector3d h4, double d2) {
  Eigen::Matrix3d k1x = lieso3(k1), k2x = lieso3(k2), k3x = lieso3(k3), k4x = lieso3(k4);
  Eigen::Matrix<double, 3,2> Akp1, Akp2, Akp3, Akp4;
  Akp1 << k1x*p1, -k1x*k1x*p1;
  Akp2 << k2x*p2, -k2x*k2x*p2;
  Akp3 << k3x*p3, -k3x*k3x*p3;
  Akp4 << k4x*p4, -k4x*k4x*p4;

  Eigen::Matrix<double,2,4> A;
  A << h1.transpose()*Akp1, h2.transpose()*Akp2, h3.transpose()*Akp3, h4.transpose()*Akp4;
  Eigen::Vector2d b;
  b(0) = d1 - h1.transpose()*k1*k1.transpose()*p1 - h2.transpose()*k2*k2.transpose()*p2;
  b(1) = d2 - h3.transpose()*k3*k3.transpose()*p3 - h4.transpose()*k4*k4.transpose()*p4;
  std::cout << A << std::endl;

  // 对 A^T 进行 QR 分解，计算其零空间
  Eigen::HouseholderQR<Eigen::MatrixXd> qr;
  qr.compute(A.transpose());
  Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
  Eigen::MatrixXd Q = qr.householderQ();

  std::cout << "Q = \n" << Q << std::endl;
  std::cout << "R = \n" << R << std::endl;

  // Q 的后两列即为 A 行向量的零空间
  Eigen::Vector<double,4> xmin = A.transpose()*(A*A.transpose()).inverse()*b;
  std::cout << "xmin = " << xmin.transpose() << std::endl;
  std::cout << "|xmin| = " << xmin.norm() << std::endl;

  return {{}};
}

bool is_ellipse(const std::vector<double>& k) {
  if (double tmp = k[3] - k[4]*k[4]/4; tmp < 0) {
    printf("cond1 = %.10f < 0\n", tmp);
    return false;
  }
  if (double tmp = (k[1]-k[2]*k[4]/2)*(k[1]-k[2]*k[4]/2) - 4*(k[0]-k[2]*k[2]/4)*(k[3]-k[4]*k[4]/4); tmp < 0) {
    printf("cond2 = %.10f < 0\n", tmp);
    return false;
  }
  return true;
}

std::vector<std::vector<double>> find_intersection_of_ellipses(const std::vector<double>& a, const std::vector<double>& b) {
  if (!is_ellipse(a) || !is_ellipse(b)) {
    return {{}};
  }

  std::vector<double> c, d, e;
  for (size_t i=0; i<5; ++i) {
    d[i] = a[i] - b[i];
  }
  c[0] = a[0] - a[2] * a[2] / 4, c[1] = a[1] - a[2] * a[4] / 2, c[2] = a[3] - a[4] * a[4] / 4;
  e[0] = d[0] - a[2] * d[2] / 2, e[1] = d[1] - (a[2] * d[4] + a[4] * d[2]) / 2, e[2] = d[3] - a[4] * d[4] / 2;
  double xbar = -d[2]/d[4], exbar = e[0] + e[1]*xbar + e[2]*xbar*xbar;

  // 交点坐标
  std::vector<Eigen::Vector2d> intersection;
  // 相交(T), 相切(F)
  std::vector<bool> isTransverse;
  intersection.resize(4), isTransverse.reserve(4);

  // Case1: d4 != 0, e(xbar) != 0
  if (!is_near_zero(d[4]) && !is_near_zero(exbar)) {
    // 四次方程系数
    std::vector<double> f(4);
    f[0] = c[0]*d[2]*d[2] + e[0]*e[0];
    f[1] = c[1]*d[2]*d[2] + 2*(c[0]*d[2]*d[4] + e[0]*e[1]);
    f[2] = c[2]*d[2]*d[2] + c[0]*d[4]*d[4] + e[1]*e[1] + 2*(c[1]*d[2]*d[4]+e[0]*e[2]);
    f[3] = c[1]*d[4]*d[4] + 2*(c[2]*d[2]*d[4] + e[1]*e[2]);
    f[4] = c[2]*d[4]*d[4] + e[2]*e[2];

    // root, multiplicity
    std::map<double, int> rmMap = solve_quartic_equation(f);
    for (const auto& rm : rmMap) {
      double x = rm.first;
      size_t multiplicity = rm.second;
      double w = -(e[0] + x*(e[1]+x*e[2])/(d[2]+d[4]*x));
      double y = w - (a[2]+x*a[4])/2;
      intersection.push_back({x,y});
      isTransverse.push_back(multiplicity == 1);
    }
  }

  // Case2: d4 != 0, e(xbar) = 0
  else if (!is_near_zero(d[4]) && is_near_zero(exbar)) {
    // Compute intersections of x=xbar with ellipse
    double ncbar = -(c[0] + xbar * (c[1] + xbar * c[2])), translate, w, y;
    if (ncbar >= 0) {
      translate = (a[2] + xbar * a[4]) / 2;
      w = sqrt(ncbar);
      y = w - translate;
      intersection.push_back({xbar, y});
      if (w > 0) {
        isTransverse.push_back(true);
        y = -w - translate;
        intersection.push_back({xbar, y});
        isTransverse.push_back(true);
      } else {
        isTransverse.push_back(false);
      }
    }
    // Compute intersections of w=-h(x) with ellipse
    std::vector<double> h(2), f(3);
    h[1] = e[2]/d[4];
    h[0] = (e[1] - d[2]*h[1]) / d[4];
    f = {c[0] + h[0]*h[0], c[1] + 2*h[0]*h[1], c[2] + h[1] * h[1]};

    // root, multiplicity
    std::map<double, int> rmMap = solve_quartic_equation(f);
    for (const auto& rm : rmMap) {
      double x = rm.first;
      translate = (a[2] + xbar*a[4]) / 2;
      y = -(h[0] + x*h[1]) - translate;
      intersection.push_back({x,y});
      isTransverse.push_back(rm.second == 1);
    }
  }

  // Case3: d4 = 0, d2 != 0, e2 != 0
  else if (is_near_zero(d[4]) && !is_near_zero(d[2]) && !is_near_zero(e[2])) {
    std::vector<double> f(5);
    f[0] = c[0]*d[2]*d[2] + e[0]*e[0];
    f[1] = c[1]*d[2]*d[2] + 2*e[0]*e[1];
    f[2] = c[2]*d[2]*d[2] + e[1]*e[1] + 2*e[0]*e[2];
    f[3] = 2*e[1]*e[2];
    f[4] = e[2]*e[2];

    // root, multiplicity
    std::map<double, int> rmMap = solve_quartic_equation(f);
    for (const auto& rm : rmMap) {
      double x = rm.first;
      double w = -(e[0] + x*(e[1]+x*e[2])) / d[2];
      double y = w - (a[2]+x*a[4])/2;
      intersection.push_back({x,y});
      isTransverse.push_back(rm.second == 1);
    }
  }

  // Case4: d4 = 0, d2 != 0, e2 = 0
  else if (is_near_zero(d[4]) && !is_near_zero(d[2]) && is_near_zero(e[2])) {
    std::vector<double> f(3);
    f[0] = c[0]*d[2]*d[2] + e[0]*e[0];
    f[1] = c[1]*d[2]*d[2] + 2*e[0]*e[1];
    f[2] = c[2]*d[2]*d[2] + e[1]*e[1];

    // root, multiplicity
    std::map<double, int> rmMap = solve_quartic_equation(f);
    for (const auto& rm : rmMap) {
      double x = rm.first;
      double w = -(e[0] + x*e[1]) / d[2];
      double y = w - (a[2]+x*a[4])/2;
      intersection.push_back({x,y});
      isTransverse.push_back(rm.second == 1);
    }
  }

  // Case5: d4 = 0, d2 = 0, e2 = 0
  else if (is_near_zero(d[4] && is_near_zero(d[2])) && is_near_zero(e[2])) {
    double xhat = -e[0] / e[1] ,nchat = -(c[0]+xhat*(c[1]+xhat*c[2]));
    if (nchat > 0) {
      double translate = (a[2] + xhat*a[4]) / 2;
      double y = sqrt(nchat) - translate;
      intersection.push_back({xhat, y});
      isTransverse.push_back(true);
      y = -sqrt(nchat) - translate;
      intersection.push_back({xhat, y});
      isTransverse.push_back(true);
    } else if (is_near_zero(nchat)) {
      double y = -(a[2] + xhat*a[4]) / 2;
      intersection.push_back({xhat, y});
      isTransverse.push_back(false);
    }
  }

  else if (is_near_zero(d[4] && is_near_zero(d[2])) && !is_near_zero(e[2])) {
    std::vector<double> f{e[0]/e[2], e[1]/e[2]};
    double mid = -f[1] / 2, discr = mid * mid - f[0];

    if (discr > 0) {
      // The roots are xhat = mid + s*sqrtDiscr for s in {-1, 1}
      double sqrtDiscr = sqrt(discr);
      std::vector<double> g{c[0] - c[2]*f[0], c[1]-c[2]*f[1]};
      if (g[1] > 0) {
        // We need s*sqrtDiscr <= -g0/g1 + f1/2
        double r = -g[0]/g[1] - mid;
        if (r >= 0) {
          double rsqr = r*r;
          if (discr < rsqr) {

          }
        }
      }
    }
  }

  return {{}};
}

std::map<double, int> solve_quadratic_equation(const std::vector<double>& k) {
  std::map<double, int> ans;
  std::complex<double> delta = sqrt(std::complex<double>(k[1]*k[1] - 4*k[2]*k[0]));
  std::vector<std::complex<double>> root(2);
  root[0] = (-k[1] + delta) / (2*k[2]);
  root[1] = (-k[1] - delta) / (2*k[2]);

  for (std::complex<double> sol : root) {
    // 过滤虚数解
    if (!is_near_zero(sol.imag(), 1e-15))
      continue;
    double x = sol.real();
    // 记录根和重根个数
    if (ans.find(x) == ans.end()) {
      ans.insert({x,1});
    } else {
      ans[x]++;
    }
  }
  return ans;
}

std::map<double, int> solve_cubic_equation(const std::vector<double>& k) {
  double d0 = k[2]*k[2] - 3*k[3]*k[1], d1 = 2*k[2]*k[2]*k[2] - 9*k[3]*k[2]*k[1] + 27*k[3]*k[3]*k[0];
  std::complex<double> d2 = sqrt(std::complex<double>(d1*d1 - 4*d0*d0*d0));
  std::map<double, int> ans;

  // d0 = d1 = 0
  if (std::norm(d0) < 1e-15 && std::norm(d1) < 1e-15) {
    ans.insert({-k[2]/(3*k[3]), 3});
    return ans;
  }

  // cubic root of complex: https:stackoverflow.com/questions/40439739
  std::complex<double> C, d3 = (std::norm(d1+d2) < 1e-12) ? (d1 - d2) / 2.0 : (d1 + d2) / 2.0;
  if (d3.real() > 0) {
    C = -pow(-d3, 1.0/3.0);
  } else {
    C = pow(d3, 1.0/3.0);
  }

  std::complex<double> e0(-0.5, sqrt(3)/2.0), e1(-0.5, -sqrt(3)/2.0);
  std::vector<std::complex<double>> root(3), cooe = {1.0, e0, e1};
  for (size_t i=0; i<root.size(); ++i) {
    root[i] = -1/(3*k[3])*(k[2] + cooe[i]*C + d0/(cooe[i]*C));
    // std::cout << "root = " << root[i] << std::endl;
  }

  for (std::complex<double> sol : root) {
    // 过滤虚数解
    if (fabs(sol.imag()) > 1e-15)
      continue;
    double x = sol.real();
    // 记录根和重根个数
    if (ans.find(x) == ans.end()) {
      ans.insert({x,1});
    } else {
      ans[x]++;
    }
  }
  return ans;
}

std::map<double, int> solve_quartic_equation(const std::vector<double>& k) {
  std::map<double, int> ans;
  std::vector<std::complex<double>> root;
  root.reserve(4);

  // Convert to a depressed quartic
  double a, b, c;
  if (k[3] == 0) {
    a = k[2]/k[4];
    b = k[1]/k[4];
    c = k[0]/k[4];
  } else {
    a = -3*k[3]*k[3]/(8*k[4]*k[4]) + k[2]/k[4];
    b = k[3]*k[3]*k[3]/(8*k[4]*k[4]*k[4]) - k[3]*k[2]/(2*k[4]*k[4]) + k[1]/k[4];
    c = -3*k[3]*k[3]*k[3]*k[3]/(256*k[4]*k[4]*k[4]*k[4]) + k[2]*k[3]*k[3]/(16*k[4]*k[4]*k[4]) - k[3]*k[1]/(4*k[4]*k[4]) + k[0]/k[4];
  }

  // Biquadratic equation
  if (b == 0) {
    std::cout << "b = 0" << std::endl;
    std::complex<double> delta = sqrt(a*a - 4*a*c);
    for (std::complex<double> tmp : {(-a + delta) / 2.0, (-a - delta) / 2.0}) {
      root.push_back(sqrt(tmp));
      root.push_back(-sqrt(tmp));
    }
  }

  // General case
  else if (b != 0) {
    std::cout << "b != 0" << std::endl;
    std::map<double, int> cubicRoot = solve_cubic_equation({4*a*c-b*b, -8*c, -4*a, 8});
    double y = cubicRoot.begin()->first;
    // double p = -a*a/12 - c, q = -a*a*a/108 + a*c/3 - b*b/8;
    // double w = std::cbrt(-q/2 + sqrt(q*q/4 + p*p*p/27)), y = a/6 + w - p/(3*w);

    double exchange = k[3]/(4*k[4]);
    std::complex<double> delta = sqrt(std::complex<double>(2*y-a));
    std::vector<std::complex<double>> D{-2*y-a + 2*b/delta, -2*y-a - 2*b/delta};
    root.push_back((-delta + sqrt(D[0])) / 2.0 - exchange);
    root.push_back((-delta - sqrt(D[0])) / 2.0 - exchange);
    root.push_back((delta + sqrt(D[1])) / 2.0 - exchange);
    root.push_back((delta - sqrt(D[1])) / 2.0 - exchange);

    for (std::complex<double> sol : root) {
      // 过滤虚数根
      if (fabs(sol.imag()) > 1e-12)
        continue;
      double x = sol.real();
      // 记录根和重根个数
      if (ans.find(x) == ans.end()) {
        ans.insert({x,1});
      } else {
        ans[x]++;
      }
    }
  }

  return ans;
}

