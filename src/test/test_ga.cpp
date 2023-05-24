#include "heuristics/genetic_algo.h"

#include "user_interface/user_interface.h"

// TSP 问题建模
int main(int argc, char **argv) {
  std::cout << "===> Genetic Algorithm begin:" << std::endl;
  // 模型参数
  const int numSalemen = 4, numNode = 25;
  MTSPModel mtsp(numNode, numSalemen);

  // 生成随机点
  Eigen::MatrixXd begPoints = get_random_matrix(3, numSalemen, 0, 20);
  Eigen::MatrixXd viaPoints = get_random_matrix(3, numNode, 0, 10);
  std::cout << "Depots:\n" << begPoints << "\n" << std::endl;
  std::cout << "Cities:\n" << viaPoints << "\n" << std::endl;
  for (int i=0; i<numNode; ++i) mtsp.city[i] = viaPoints.col(i);
  for (int i=0; i<numSalemen; ++i) mtsp.depot[i] = begPoints.col(i);

  // 计算代价矩阵
  mtsp.calc_cost_matrix();
  std::cout << mtsp.costMat << std::endl;

  std::cout << "\n===> Initial Generation:" << std::endl;
  // 模型参数
  int groupSize = 200, generations = 1e3;
  double crossoverProb = 0.2;

  // 初始化种群
  mtsp.ga_init_population(groupSize, generations, crossoverProb);
  for (int i=0; i<5; ++i) {
    std::cout << mtsp.population[i] << std::endl;
  }
  std::cout << "..." << std::endl;

  // 评估个体适应性
  std::cout << "\n===> Fitness function:" << std::endl;
  mtsp.ga_evaluate_fitness();

  std::cout << "minCost = " << mtsp.minCost << std::endl;
  std::cout << "maxCost = " << mtsp.maxCost << std::endl;
  std::cout << "meanCost = " << mtsp.sumCost/groupSize << std::endl;

  std::cout << "\n===> Begin iteration:" << std::endl;

  int mateSize = 2*floor(groupSize*crossoverProb);
  // 迭代次数
  for (int ite=0; ite<1e4; ++ite) {
    // 根据适应度生成轮盘
    mtsp.ga_update_roulette_wheel();

    // 交叉
    int idxMon = mtsp.ga_roll_wheel(), idxDad = mtsp.ga_roll_wheel();
    Indivisual sister = mtsp.ga_crossover_tcx(mtsp.population[idxMon], mtsp.population[idxDad]);
    mtsp.ga_evaluate_fitness(sister);
    Indivisual brother = mtsp.ga_crossover_tcx(mtsp.population[idxMon], mtsp.population[idxDad]);
    mtsp.ga_evaluate_fitness(brother);

    // 替换
    int maxParent = mtsp.population[idxMon].fitness > mtsp.population[idxDad].fitness ? idxMon : idxDad;
    if (sister.fitness < brother.fitness) {
      mtsp.population[maxParent] = sister;
    } else {
      mtsp.population[maxParent] = brother;
    }

    // 更新最优解
    if (mtsp.population[maxParent].fitness < mtsp.optimumInd.fitness) {
      mtsp.optimumInd = mtsp.population[maxParent];
    }
  }

  std::cout << mtsp.optimumInd << std::endl;















  // 保存节点数量、访问顺序、分配序列
  std::ofstream resultFile("build/data/qkhull_ga_result", std::ios::trunc);
  resultFile << numNode << "\n" << numSalemen << std::endl;
  for (int i=0; i<mtsp.optimumInd.firstGene.size(); ++i) {
    resultFile << mtsp.optimumInd.firstGene[i] << std::endl;
  }
  for (int i=0; i<mtsp.optimumInd.secondGene.size(); ++i) {
    resultFile << mtsp.optimumInd.secondGene[i] << std::endl;
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
    finPointF << viaPoints.col(mtsp.optimumInd.firstGene[i]).transpose() << std::endl;
  }

  return 0;
}
