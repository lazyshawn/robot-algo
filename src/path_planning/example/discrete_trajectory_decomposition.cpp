#include "user_interface/user_interface.h"
#include "path_planning/curve_decomposition.h"

// 配置参数
// 直线近似误差
const double linearErr = 2;
// 曲率分割误差
const double curveErr = 2e-1;
// 数据存放路径
const std::string dataPath = "data/curve_decomposition/";
// 文件保存路径
const std::string savePath = "build/data/curve_decomposition/";

// 读取数据的文件名称
std::string dataFile;
// 数据点
Eigen::MatrixXd pntMat;
// 数据点
std::vector<Eigen::Vector3d> fitPnt;
// 离散轨迹类
DiscreteTrajectory traj;

int discrete_trajectory_decomposition(int argc, char** argv);
void discrete_trajectory_transition();
// 结果可视化
void decomposition_visualization();
// 贝塞尔曲线过渡
void bezier_transition(std::list<Eigen::Vector3d>& nodePnt, std::list<Eigen::Vector<double,7>>& arcInfo, double smooth);

int main(int argc, char** argv) {
  // discrete_trajectory_decomposition(argc, argv);
  // discrete_trajectory_transition();

  // Eigen::Vector3d op1 = (beg - cen).normalized(), op2 = (mid - cen).normalized(), op3 = (end - cen).normalized();
  // std::cout << "op1 = " << op1.transpose() << std::endl;
  // std::cout << "op3 = " << op3.transpose() << std::endl;
  // std::cout << "norm = " << (op1.cross(op3)).transpose() << std::endl;
  // std::cout << (85.68) * M_PI / 180 << std::endl;

  return 0;
}

void discrete_trajectory_transition() {
  std::list<Eigen::Vector3d> nodePnt;
  std::list<Eigen::Vector<double,7>> arcInfo;

  nodePnt.push_back({0, 0, 0});
  nodePnt.push_back({1, 0, 0});
  nodePnt.push_back({1, 0, 1});
  arcInfo.push_back({0, 0, 0, -1, 0, 0, 0});
  arcInfo.push_back({1, 0, 0.5, 0.5, 0, M_PI, 0});

  std::ofstream nodePntFile(savePath + "node_pnt", std::ios::trunc);
  for (const auto& node : nodePnt) {
    nodePntFile << node.transpose() << std::endl;
  }
  bezier_transition(nodePnt, arcInfo, 0.2);

  std::ofstream arcInfoFile(savePath + "arc_info", std::ios::trunc);
  for (const auto& arc : arcInfo) {
    arcInfoFile << arc.transpose() << std::endl;
  }

  std::cout << "After ================> " << std::endl;
  std::ofstream transPntFile(savePath + "trans_pnt", std::ios::trunc);
  for (auto& pnt : nodePnt) {
    transPntFile << pnt.transpose() << std::endl;
  }
  return;
}

int discrete_trajectory_decomposition(int argc, char** argv) {
  dataFile = argc > 1 ? argv[1] : "final_22.txt";
  std::cout << "Read data from: " << dataFile << std::endl;

  if (int flag = read_eigen_from_file(dataPath + dataFile, pntMat); flag > 0) {
    std::cout << "Read data failed with code: " << flag << std::endl;
    return 1;
  }
  pntMat.transposeInPlace();

  // 以向量格式保存拟合点
  fitPnt = std::vector<Eigen::Vector3d>(pntMat.cols());
  for (int i=0; i<pntMat.cols(); ++i) {
    fitPnt[i] = pntMat.col(i);
  }
  traj.pntList = fitPnt;

  decomposition_visualization();

  traj.perform_trajectory_decomposition(linearErr, curveErr);
  traj.calc_trajectory_pose({0,0,1});

  std::list<Eigen::Vector3d> nodePnt;
  std::list<Eigen::Vector<double,7>> arcInfo;
  traj.export_trajectory(nodePnt, arcInfo);

  for (auto& pnt : nodePnt) {
    std::cout << pnt.transpose() << std::endl;
  }

  bezier_transition(nodePnt, arcInfo, 20);

  std::cout << "After ================> " << std::endl;
  std::ofstream transPntFile(savePath + "trans_pnt", std::ios::trunc);
  for (auto& pnt : nodePnt) {
    transPntFile << pnt.transpose() << std::endl;
  }

  return 0;
}

void decomposition_visualization() {
  // Blurred segment splition for midpoitn curve -> dominant points
  std::vector<size_t> dpIdx = maximal_blurred_segment_split(fitPnt, linearErr);
  // Get midpoint curve of the maximal blurred segment
  std::vector<Eigen::Vector3d> mpc =  convert_into_tangent_space(fitPnt, dpIdx);
  // Blurred segment splition for midpoitn curve
  std::vector<size_t> idxList = maximal_blurred_segment_split(mpc, curveErr);
  // 圆弧和直线段分隔点的索引
  std::vector<size_t> sepIdx = std::vector<size_t>(idxList.size(), dpIdx.back());
  for (size_t i=0; i<idxList.size()-1; ++i) {
    sepIdx[i] = dpIdx[idxList[i]];
  }
  // 逐段进行圆弧近似
  std::vector<std::pair<size_t, double>> pair = arc_approximation(fitPnt, sepIdx);

  // Mid point curve: 累计弦长 - 累计夹角
  std::ofstream mpcFile(savePath + "mpc", std::ios::trunc);
  for (size_t i=0; i<mpc.size(); ++i) {
    mpcFile << mpc[i][0] << " " << mpc[i][1] << std::endl;
  }
  // MPC 曲线中 DP 点的索引
  std::ofstream mpcDpFile(savePath + "mpc_dp_idx", std::ios::trunc);
  for (size_t i=0; i<idxList.size(); ++i) {
    mpcDpFile << idxList[i] << std::endl;
  }
  // 分割点索引
  std::ofstream sepIdxFile(savePath + "sep_idx", std::ios::trunc);
  for (size_t i=0; i<sepIdx.size(); ++i) {
    sepIdxFile << sepIdx[i] << std::endl;
  }
  // 圆弧点索引
  std::ofstream apIdxFile(savePath + "arc_point_idx");
  for (auto& p : pair) {
    apIdxFile << p.first << std::endl;
  }
}

void bezier_transition(std::list<Eigen::Vector3d>& nodePnt, std::list<Eigen::Vector<double,7>>& arcInfo, double smooth) {
  // 轨迹节点迭代器
  std::list<Eigen::Vector3d>::iterator endIte = nodePnt.begin();
  std::list<Eigen::Vector3d>::iterator begIte = endIte++;
  // 轨迹信息迭代器
  std::list<Eigen::Vector<double,7>>::iterator infoIte = arcInfo.begin();

  // 上一段轨迹终点, 下一段轨迹的起点处的过渡点
  Eigen::Vector3d preMark = *begIte, aftMark(0,0,0);
  // 过渡点处的切线方向
  Eigen::Vector3d preNorm(0,0,0), aftNorm(0,0,0);
  // 贝塞尔曲线过渡段长度
  double tranLen = 0.0;

  // 记录第一段轨迹点终点信息
  Eigen::Vector3d begPnt = *begIte, endPnt = *endIte;
  Eigen::Vector<double,7> info = *infoIte;
  if (info(3) > 1000 || info(3) < 0) {
    preNorm = (endPnt - begPnt).normalized();
    preMark = endPnt - preNorm*smooth;
  } else {
    // 切向量, 圆心
    Eigen::Vector3d normal = info.tail(3).normalized(), center = info.head(3);
    // 半径
    double radius = (center - begPnt).norm();
    // 过渡圆弧对应的圆心角
    double ang = smooth / radius;
    // 过渡点位置
    preMark = Eigen::AngleAxisd(-ang, normal) * (endPnt - center) + center;
    // 切线方向
    preNorm = normal.cross((preMark - center).normalized());
  }
  tranLen = smooth * preNorm.dot((endPnt - preMark).normalized());

  // 遍历每一段轨迹
  infoIte++;
  for (auto curInfoIte = infoIte; curInfoIte != arcInfo.end(); ++curInfoIte) {
    // 更新迭代器
    begIte = endIte++;
    // infoIte++;
    // 更新轨迹信息
    begPnt = *begIte, endPnt = *endIte;
    info = *curInfoIte;

    // 计算起点处的过渡点位置和切向量
    if (info(3) > 1000 || info(3) < 0) {
      // 起点处的过渡点
      aftNorm = (endPnt - begPnt).normalized();
      // 过渡点处的切向量
      aftMark = begPnt + aftNorm*smooth;
    } else {
      // 切向量, 圆心
      Eigen::Vector3d normal = info.tail(3).normalized(), center = info.head(3);
      // 半径
      double radius = (center - begPnt).norm();
      // 过渡圆弧对应的圆心角
      double ang = smooth / radius;
      // 过渡点位置
      aftMark = Eigen::AngleAxisd(ang, normal) * (begPnt - center) + center;
      // 切线方向
      aftNorm = normal.cross((aftMark - center).normalized());
    }
    tranLen = std::min(tranLen, smooth * aftNorm.dot((aftMark - begPnt).normalized()));

    double vel = 0.01, freq = 1;
    // 计算过渡曲线
    BezierCurve bezier;
    bezier.construct_from_vct(preMark, preNorm, aftMark, aftNorm, 0.3*tranLen);
    std::vector<double> paraVec;
    bezier.discrete_arc_length(paraVec, 0, 1, 1e-2);
    std::vector<double> cumuChordLength;
    double curveLen = bezier.get_chord_length(paraVec, cumuChordLength);
    size_t numSegm = std::ceil(curveLen / vel * freq);
    std::vector<double> samplePara = bezier.get_uniform_sample(paraVec, cumuChordLength, curveLen / numSegm);

    // 修改上一段终点
    *begIte = bezier.get_point(samplePara[0]);
    // *begIte = preMark;
    // nodePnt.insert(endIte, aftMark);
    // arcInfo.insert(curInfoIte, {0,0,0,-1,0,0,0});
    // 修改旋转角度
    // 插入过渡线段
    for (size_t i=1; i<samplePara.size(); ++i) {
      nodePnt.insert(endIte, bezier.get_point(samplePara[i]));
      arcInfo.insert(infoIte, {0,0,0,-1,0,0,0});
    }

    // 更新终点处的过渡点和切线方向
    if (info(3) > 1000 || info(3) < 0) {
      preNorm = (endPnt - begPnt).normalized();
      preMark = endPnt - preNorm*smooth;
    } else {
      // 切向量, 圆心
      Eigen::Vector3d normal = info.tail(3).normalized(), center = info.head(3);
      // 半径
      double radius = (center - begPnt).norm();
      // 过渡圆弧对应的圆心角
      double ang = smooth / radius;
      // 过渡点位置
      preMark = Eigen::AngleAxisd(-ang, normal) * (endPnt - center) + center;
      // 切线方向
      preNorm = normal.cross((preMark - center).normalized());
    }
    tranLen = smooth * preNorm.dot((endPnt - preMark).normalized());
  }
}

