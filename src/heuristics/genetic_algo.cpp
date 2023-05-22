#include "heuristics/genetic_algo.h"
#include <iostream>

std::vector<int> get_random_permutation(int num) {
  // 1~n 的初始队列
  std::vector<int> array(num);
  for (int i = 0; i < num; ++i) {
    array[i] = i;
  }
  // 获取随机种子
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // 生成随机队列
  std::shuffle(array.begin(), array.end(), std::default_random_engine(seed));
  return array;
}

int get_uniform(int num) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> uniform(1, num);
  return uniform(generator);
}

std::vector<int> get_sequence_with_sum(int sum, int numPartion) {
  std::vector<int> ans(numPartion), array;

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> uniform(1, sum-1);

  // 生成 n-1 个 (0,m) 的随机数
  for (int i=0; i<numPartion-1; ++i) {
    array.push_back(uniform(generator));
  }
  array.push_back(0);
  array.push_back(sum);
  std::sort(array.begin(), array.end());

  // 数组 array 中相邻数的差即为所求
  for (int i=0; i<numPartion; ++i) {
    ans[i] = array[i+1] - array[i];
  }
  return ans;
}

void append_sequence(std::vector<std::vector<int>> &array, int numPartion) {
  int arraySize = array.size(), totalNode = array[0].size();

  // 生成随机数
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> uniform(1, totalNode);

  // 添加分配序列
  std::vector<int> sum(arraySize);
  for (int i = 0; i < numPartion - 1; ++i) {
    for (int j = 0; j < arraySize; ++j) {
      int tmp = (uniform(generator)) % (totalNode - sum[j]);
      array[j].push_back(tmp);
      sum[j] += tmp;
    }
  }
  for (int i = 0; i < arraySize; ++i) {
    int tmp = (totalNode - sum[i]);
    array[i].push_back(tmp);
  }
}

Indivisual::Indivisual(int numFirstGene, int numSecondGene) {
  firstGene = std::vector<int>(numFirstGene);
  secondGene = std::vector<int>(numSecondGene);
}

void Indivisual::setFitness(double tmpFitness) { fitness = tmpFitness; }
