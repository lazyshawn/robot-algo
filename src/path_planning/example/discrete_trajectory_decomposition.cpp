#include "user_interface/user_interface.h"
#include "path_planning/curve_decomposition.h"

// 配置参数
// 直线近似误差
const double linearErr = 8e-2;
// 曲率分割误差
const double curveErr = 8e-2;
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

// 结果可视化
void decomposition_visualization();

int main(int argc, char** argv) {
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

  // for (size_t i=0; i<traj.pntPose.size(); ++i) {
  //   std::cout << "i = " << i << std::endl;
  //   std::cout << traj.pntPose[i].matrix() << std::endl;
  // }

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

