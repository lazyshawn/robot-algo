#pragma once

#include <algorithm>
#include <random>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>

// 获取从 1~num 的随机排列
std::vector<int> get_random_permutation(int num);

// 获取从 1~num 的均匀分布随机数
int get_uniform(int num);

std::vector<int> get_sequence_with_sum(int sum, int numPartion);

void append_sequence(std::vector<std::vector<int>>& array, int numPartion);

class Indivisual {
public:
  // 访问顺序
  std::vector<int> firstGene;
  // 分配情况
  std::vector<int> secondGene;
  // 适应度
  double fitness = 0;

public:
  Indivisual(){};
  Indivisual(int numFirstGene, int numSecondGene);
  // 计算个体适应度
  void setFitness(double fitness);
};

class GA_programmer {
public:
  std::vector<Indivisual> population;
};

