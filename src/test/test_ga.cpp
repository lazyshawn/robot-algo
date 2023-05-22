#include "heuristics/genetic_algo.h"

#include "user_interface/user_interface.h"

// TSP 问题建模
int main(int argc, char **argv) {
  std::cout << "===> Genetic Algorithm begin:" << std::endl;
  // 模型参数
  const int numSalemen = 4, numNode = 15;

  // 生成随机点，前m个视为商人起点
  Eigen::MatrixXd begPoints = get_random_matrix(3, numSalemen, 0, 20);
  Eigen::MatrixXd viaPoints = get_random_matrix(3, numNode, 0, 10);
  std::cout << "Depots:\n" << begPoints << "\n" << std::endl;
  std::cout << "Cities:\n" << viaPoints << "\n" << std::endl;

  // 计算代价矩阵
  Eigen::MatrixXd costMat = Eigen::MatrixXd::Identity(numNode, numNode) * 999;
  double maxDist=-1.0, minDist=999;
  std::cout << "Cost Matrix:" << std::endl;
  for (int i = 0; i < viaPoints.cols(); ++i) {
    for (int j = i + 1; j < viaPoints.cols(); ++j) {
      double dist = (viaPoints.col(i) - viaPoints.col(j)).norm();
      costMat(i, j) = costMat(j, i) = dist;
    }
  }
  std::cout << costMat << std::endl;

  std::cout << "\n===> Initial Generation:" << std::endl;
  // 模型参数
  int groupSize = 200, generations = 1e5;
  double elimatePro = 0.8;

  // 初始种群
  std::vector<Indivisual> population(groupSize, Indivisual(numNode, numSalemen));
  // 生成访问顺序
  for (int i = 0; i < groupSize; ++i) {
    population[i].firstGene = get_random_permutation(numNode);
    population[i].secondGene = get_sequence_with_sum(numNode, numSalemen);
  }

  for (int i=0; i<5; ++i) {
    for (int j=0; j<population[0].firstGene.size(); ++j) {
      std::cout << population[i].firstGene[j] << ", ";
    }
    std::cout << " --  ";
    for (int j=0; j<population[0].secondGene.size(); ++j) {
      std::cout << population[i].secondGene[j] << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << "..." << std::endl;

  // 评估个体适应性
  std::cout << "\n===> Fitness function:" << std::endl;
  for (int i=0; i<groupSize; ++i) {
    int begNodeIdx = 0, salemenIdx = 0;
    for (int j=0; j<numNode; ++j) {
      // 当前城市是商人出发后的第一个城市
      if (j == begNodeIdx) {
        population[i].fitness += (viaPoints.col(population[i].firstGene[j]) - begPoints.col(salemenIdx)).norm();
        // 跳过访问0城市的商人
        do {
        // 下个商人的第一个城市的索引
          begNodeIdx += population[i].secondGene[salemenIdx];
          salemenIdx++;
        } while (population[i].secondGene[salemenIdx] == 0);
      }
      else {
        population[i].fitness += costMat(population[i].firstGene[j-1],population[i].firstGene[j]);
      }
    }
  }

  double minCost = 999, maxCost = -1, sumCost = 0, meanCos, tmpCost;
  std::vector<int> optimum_first(numNode), optimum_second(numSalemen);
  for (int i=0; i<groupSize; ++i) {
    tmpCost = population[i].fitness;
    maxCost = std::max(maxCost, tmpCost);
    sumCost += tmpCost;
    if (tmpCost < minCost) {
      minCost = tmpCost;
      optimum_first = population[i].firstGene;
      optimum_second = population[i].secondGene;
    }
  }
  std::cout << "minCost = " << minCost << std::endl;
  std::cout << "maxCost = " << maxCost << std::endl;
  std::cout << "meanCost = " << sumCost/groupSize << std::endl;

  std::cout << "\n===> Begin iteration:" << std::endl;
  // 根据适应度生成轮盘


  // 保存节点数量、访问顺序、分配序列
  std::ofstream resultFile("build/data/qkhull_ga_result", std::ios::trunc);
  resultFile << numNode << "\n" << numSalemen << std::endl;
  for (int i=0; i<optimum_first.size(); ++i) {
    resultFile << optimum_first[i] << std::endl;
  }
  for (int i=0; i<optimum_second.size(); ++i) {
    resultFile << optimum_second[i] << std::endl;
  }

  // 保存原始点(与点序列对应)
  std::ofstream rawPointF("build/data/qkhull_ga_raw_points", std::ios::trunc);
  for (int i = 0; i < numSalemen; ++i) {
    rawPointF << begPoints.col(i).transpose() << std::endl;
  }
  for (int i = 0; i < numNode; ++i) {
    rawPointF << viaPoints.col(i).transpose() << std::endl;
  }

  // 按访问顺序保存随机点
  std::ofstream finPointF("build/data/qkhull_ga_seq_points", std::ios::trunc);
  for (int i = 0; i < numSalemen; ++i) {
    finPointF << begPoints.col(i).transpose() << std::endl;
  }
  for (int i = 0; i < numNode; ++i) {
    finPointF << viaPoints.col(optimum_first[i]).transpose() << std::endl;
  }

  return 0;
}
