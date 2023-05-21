#include "heuristics/genetic_algo.h"

std::vector<int> get_random_permutation(int num) {
  // 1~n 的初始队列
  std::vector<int> array(num);
  for (int i=0; i<num; ++i) {
    array[i] = i+1;
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
  std::uniform_int_distribution<int> uniform(1,num);
  return uniform(generator);
}

