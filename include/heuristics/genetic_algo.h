#pragma once

#include <algorithm>
#include <random>
#include <vector>
#include <chrono>
#include <limits>
#include <iostream>

#include <eigen3/Eigen/Dense>

// 获取从 1~num 的随机排列
std::vector<int> get_random_permutation(int num);

// 获取从 1~num 的均匀分布随机数
std::vector<int> get_uniform(int num, int size);

std::vector<int> get_sequence_with_sum(int sum, int numPartion);

void append_sequence(std::vector<std::vector<int>>& array, int numPartion);

class Indivisual {
public:
  // 访问顺序
  std::vector<int> firstGene;
  std::vector<int> index;
  // 分配情况
  std::vector<int> secondGene;
  // 适应度
  double fitness = 0.0;
  double wheelPartion = 0.0;

public:
  Indivisual(){};
  Indivisual(int numFirstGene, int numSecondGene);
  Indivisual& operator= (const Indivisual& ind);
  // 计算个体适应度
  void setFitness(double fitness);
  friend std::ostream &operator<<(std::ostream &output, const Indivisual &ind);
};

class MTSPModel {
public:
  int numCity, numSalemen;
  std::vector<Eigen::Vector3d> depot, city;
  Eigen::MatrixXd costMat;
  double minCost, maxCost;

  // GA related
  int groupSize, generations;
  double crossoverProb, mutationProb;
  // population
  std::vector<Indivisual> population;
  Indivisual optimumInd;
  std::vector<double> rouletteWheel;
  double minFitness, maxFitness, sumFitness;
  std::vector<int> selection_pool, replacement_pool;

public:
  MTSPModel(int numCity_, int numSalemen_);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void calc_cost_matrix();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_init_population(int groupSize_, int generations_, double crossoverProb_, double mutationProb_);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_random_indivisual(Indivisual& ind);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_fitness_statistics();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  double ga_total_dist(Indivisual ind);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  int ga_load_balance(Indivisual ind);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  double ga_evaluate_fitness(Indivisual& ind);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_evaluate_fitness();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_update_roulette_wheel();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  int ga_roll_wheel();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_crossover();
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  Indivisual ga_crossover_tcx(Indivisual mon, Indivisual dad);
  /* 
  * @brief : 
  * @param : 
  * @return: 
  */
  void ga_mutation(Indivisual& ind);
};

