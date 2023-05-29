#include "genetic_algo.h"

#include "user_interface.h"

// TSP 问题建模
int main(int argc, char **argv) {
  std::cout << "===> Genetic Algorithm begin:" << std::endl;
  // 模型参数
  const int numSalemen = 4, numNode = 25;
  MTSPModel mtsp(numNode, numSalemen);

  // 生成随机点
  Eigen::MatrixXd begPoints = get_random_matrix(3, numSalemen, 0, 20);
  begPoints << -20, -20, 20, 20,
               -20, 20, -20, 20,
               -20, -20, -20, -20;
  Eigen::MatrixXd viaPoints = get_random_matrix(3, numNode, 0, 10);
  // for (int i=0; i<viaPoints.cols(); ++i) {
  //   viaPoints.col(i)[2] = 0;
  // }
  std::cout << "Depots:\n" << begPoints << "\n" << std::endl;
  std::cout << "Cities:\n" << viaPoints << "\n" << std::endl;
  for (int i=0; i<numNode; ++i) mtsp.city[i] = viaPoints.col(i);
  for (int i=0; i<numSalemen; ++i) mtsp.depot[i] = begPoints.col(i);

  // 计算代价矩阵
  mtsp.calc_cost_matrix();
  std::cout << mtsp.costMat << std::endl;

  std::cout << "\n===> Initial Generation:" << std::endl;
  // 模型参数
  int groupSize = 200, generations = 1e5;
  double crossoverProb = 0.2, mutationProb = 0.1;
  std::vector<double> optRec;

  // 初始化种群
  mtsp.ga_init_population(groupSize, generations, crossoverProb, mutationProb);

  // 评估个体适应性
  std::cout << "\n===> Fitness function:" << std::endl;
  for (int i=0; i<groupSize; ++i) {
    mtsp.ga_evaluate_fitness(mtsp.population[i]);
  }
  mtsp.ga_fitness_statistics();

  for (int i=0; i<5; ++i) {
    std::cout << mtsp.population[i] << std::endl;
  }
  std::cout << "..." << std::endl;
  // mtsp.ga_evaluate_fitness();

  std::cout << "minCost = " << mtsp.minFitness << std::endl;
  std::cout << "maxCost = " << mtsp.maxFitness << std::endl;
  std::cout << "meanCost = " << mtsp.sumFitness/groupSize << std::endl;
  optRec.push_back(mtsp.optimumInd.fitness);

  std::cout << "\n===> Begin iteration:" << std::endl;

  // 迭代次数
  for (int ite=0; ite<generations; ++ite) {
    mtsp.ga_fitness_statistics();
    // 根据适应度生成轮盘
    mtsp.ga_update_roulette_wheel();

    // 交叉
    int idxMon = mtsp.ga_roll_wheel(), idxDad = mtsp.ga_roll_wheel();
    Indivisual sister = mtsp.ga_crossover_tcx(mtsp.population[idxMon], mtsp.population[idxDad]);
    mtsp.ga_evaluate_fitness(sister);
    Indivisual brother = mtsp.ga_crossover_tcx(mtsp.population[idxMon], mtsp.population[idxDad]);
    mtsp.ga_evaluate_fitness(brother);

    // 变异
    mtsp.ga_mutation(sister);
    mtsp.ga_mutation(brother);

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
      optRec.push_back(mtsp.optimumInd.fitness);
    }
  }

  std::cout << mtsp.optimumInd << std::endl;
  for (int i = 0; i < static_cast<int>(optRec.size()); ++i) {
    std::cout << optRec[i] << std::endl;
  }

  // 保存节点数量、访问顺序、分配序列
  std::ofstream resultFile("build/data/qkhull_ga_result", std::ios::trunc);
  resultFile << numNode << "\n" << numSalemen << std::endl;
  for (int i = 0; i < static_cast<int>(mtsp.optimumInd.firstGene.size()); ++i) {
    resultFile << mtsp.optimumInd.firstGene[i] << std::endl;
  }
  for (int i = 0; i < static_cast<int>(mtsp.optimumInd.secondGene.size()); ++i) {
    resultFile << mtsp.optimumInd.secondGene[i] << std::endl;
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
