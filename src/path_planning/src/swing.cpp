#include "path_planning/swing.h"

Eigen::Vector3d sin_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius) {
  return std::sin(phase) * offsetDir * amplitude;
}

Eigen::Vector3d circle_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius) {
  phase = - (phase + M_PI);
  Eigen::Vector3d off = amplitude * std::sin(phase) * offsetDir + radius * std::cos(phase) * dir;
  return off + dir*radius;
}

Eigen::Vector3d lemniscate_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius) {
  phase = - (phase + M_PI/2);
  double sinT = std::sin(phase), cosT = std::cos(phase);
  // 分母
  double dominate = 1 + sinT * sinT;
  double x = 2.828427124746275 * sinT * cosT / dominate, y = cosT / dominate;
  return x * radius * dir - y * amplitude * offsetDir;
}

Eigen::Vector3d L_swing(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius) {
  Eigen::Vector3d result, orth = offsetDir.cross(dir);

  // 取余
  double remainder = phase - std::trunc(phase / (2*M_PI)) * 2*M_PI - M_PI;
  double ratio = remainder / M_PI;
  if (ratio < 0) {
    ratio = ratio > -1.0/2 ? -ratio : 1+ratio;
    result = amplitude * ratio * offsetDir;
  }
  else {
    ratio = ratio < 1.0/2 ? ratio : 1-ratio;
    result = amplitude * ratio * orth;
  }

  return result;
}

std::vector<Eigen::Vector<double,6>> swing_interpolation(const std::vector<Eigen::Vector3d>& traj, SwingType type) {
  // 摆动补偿函数的函数签名
  typedef std::function<Eigen::Vector3d(double phase, Eigen::Vector3d offsetDir, Eigen::Vector3d dir, double amplitude, double radius)> swingFuncType;
  // 摆动类型与补偿函数的映射表
  static std::map<SwingType, swingFuncType> funcMap = {
    {SwingType::SIN, sin_swing},
    {SwingType::CIRCLE, circle_swing},
    {SwingType::LEMNI, lemniscate_swing},
    {SwingType::L, L_swing},
  };
  swingFuncType swingFunc = funcMap[type];

  // 上方向
  Eigen::Vector3d upper(0,0,1);
  // 总段数
  size_t num = 900;
  // 偏移振幅
  double amplitude = 0.4;
  // 偏移振幅
  double radius = 0.2;
  // 摆动频率
  double frequency = 20;

  // 单位化
  upper.normalize();

  // 结果向量: <pos, dir>
  std::vector<Eigen::Vector<double,6>> result(num+1);

  // 至少需要两个点
  if (traj.size() <2) return{};
  Eigen::Vector3d beg = traj[0], end = traj.back();

  // 直线: 不需要改变姿态
  if (traj.size() == 2) {
    // 总位移
    Eigen::Vector3d path = traj[1] - traj[0], dir = path.normalized();
    // 偏移方向
    Eigen::Vector3d offsetDir = dir.cross(upper);

    for (size_t i=0; i<result.size(); ++i) {
      // 偏移的相位角
      double phase = 2*M_PI*i/num*frequency;
      // 牵连运动
      Eigen::Vector3d ePos = beg + double(i)/num * path;
      // 相对运动
      Eigen::Vector3d rPos = swingFunc(phase, offsetDir, dir, amplitude, radius);
      result[i] << ePos + rPos, dir;
    }
  }
  // 圆弧: 均匀改变姿态
  else if (traj.size() == 3) {
    Eigen::Vector3d mid = traj[1];
    Eigen::Vector3d cen = triangular_circumcenter(beg, mid, end);
    // 当前点所在的相位角
    Eigen::Vector3d op1 = beg - cen, op2 = mid - cen, op3 = end - cen;
    // 半径过大
    if (op1.norm() > 1e3) return {};
    // 正法向: 从起点逆时针绕到终点的单位转轴
    Eigen::Vector3d norm = op1.cross(op3).normalized();
    // 夹角
    double theta = std::acos(op1.dot(op3)/op1.norm()/op3.norm());
    // 修正夹角和正法向
    if (op1.cross(op2).dot(op2.cross(op3)) <= 0) {
      theta = 2*M_PI - theta;
      norm *= -1;
    }

    for (size_t i=0; i<result.size(); ++i) {
      // 偏移方向
      Eigen::Vector3d offsetDir = Eigen::AngleAxisd(double(i)/num*theta, norm) * op1;
      // 切线方向
      Eigen::Vector3d dir = norm.cross(offsetDir).normalized();
      // 偏移的相位角
      double phase = 2*M_PI*i/num*frequency;
      // 牵连运动
      Eigen::Vector3d ePos = offsetDir + cen;
      // 相对运动
      Eigen::Vector3d rPos = swingFunc(phase, offsetDir, dir, amplitude, radius);

      result[i] << ePos + rPos, dir;
    }
  }

  return result;
}

