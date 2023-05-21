#include "heuristics/genetic_algo.h"

#include "user_interface/user_interface.h"
#include <eigen3/Eigen/Dense>

// TSP 问题建模
int main(int argc, char** argv) {
  std::cout << "===> Genetic Algorithm begin:" << std::endl;
  // 模型参数
  int numSalemen = 4, numNode = 20;

  // 生成随机点，前m个视为商人起点
  Eigen::MatrixXd pointsMat = get_random_matrix(3, numSalemen + numNode, 0, 10);
  std::cout << pointsMat << std::endl;
  // 保存随机点
  std::ofstream pointFile("build/data/qkhull_ga_points", std::ios::trunc);
  for (int i=0; i<pointsMat.cols(); ++i) {
    pointFile << pointsMat.col(i).transpose() << std::endl;
  }

  // 计算代价矩阵
  Eigen::MatrixXd costMat = Eigen::MatrixXd::Identity(numNode, numNode)*999;
  for (int i=0; i<pointsMat.cols(); ++i) {
    for (int j=i+1; j<pointsMat.cols(); ++j) {
    }
  }

  // std::cout << "===> Initial Generation:" << std::endl;
  // for (int i=0; i<100; ++i) {
  //   std::vector<int> array = get_random_permutation(10);
  //   for (int i=0; i<array.size(); ++i) {
  //     std::cout << array[i] << ", ";
  //   }
  //   std::cout << std::endl;
  // }

  // for (int i=0; i<100; ++i) {
  //   std::cout << "Random: " << get_uniform(10) << std::endl;
  // }
  return 0;
}
