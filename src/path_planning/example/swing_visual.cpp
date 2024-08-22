#include "path_planning/swing.h"
#include "user_interface/user_interface.h"

// 结果保存路径
const std::string savePath = "build/data/swing/";
// 轨迹的计算结果
std::vector<Eigen::Vector<double,6>> result;

int main(int argc, char** argv) {
  SwingType swinType = SwingType::L;
  // SwingType swinType = SwingType::SIN;

  result = swing_interpolation({{0,0,0}, {3,0,0}}, swinType);
  // result = swing_interpolation({{6,2,3}, {4,6,3}, {1,-3,3}}, swinType);

  // 输出结果
  std::ofstream posFile(savePath + "pos_dir");
  for (size_t i=0; i<result.size(); ++i) {
    posFile << result[i].transpose() << std::endl;
  }
  return 0;
}

