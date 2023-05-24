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

std::vector<int> get_uniform(int num, int size=10) {
  if (num == 0) return std::vector<int>(size, 1);
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> uniform(1, num);

  std::vector<int> randomVector(size);
  for (int i=0; i<size; ++i) {
    randomVector[i] = uniform(generator);
  }
  return randomVector;
}

double get_uniform_double() {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> uniform(0,1);
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

Indivisual& Indivisual::operator= (const Indivisual& ind) {
  firstGene = ind.firstGene;
  index = ind.index;
  // 分配情况
  secondGene = ind.secondGene;
  // 适应度
  fitness = ind.fitness;
  wheelPartion = ind.wheelPartion;
  return *this;
}

void Indivisual::setFitness(double tmpFitness) { fitness = tmpFitness; }

std::ostream &operator<<(std::ostream &output, const Indivisual &ind) {
    for (int j=0; j<ind.firstGene.size(); ++j) {
      output << ind.firstGene[j] << ", ";
    }
    output << " --  ";
    for (int j=0; j<ind.secondGene.size(); ++j) {
      output << ind.secondGene[j] << ", ";
    }
    output << " :  " << ind.fitness;
    return output;
}

MTSPModel::MTSPModel(int numCity_, int numSalemen_) {
  numCity = numCity_;
  numSalemen = numSalemen_;

  depot = std::vector<Eigen::Vector3d>(numSalemen);
  city = std::vector<Eigen::Vector3d>(numCity);
  costMat = Eigen::MatrixXd::Identity(numCity, numCity) * 999;
}


void MTSPModel::calc_cost_matrix() {
  double maxDist=-1.0, minDist=999;
  std::cout << "Cost Matrix:" << std::endl;
  for (int i = 0; i < city.size(); ++i) {
    for (int j = i + 1; j < city.size(); ++j) {
      double dist = (city[i] - city[j]).norm();
      costMat(i, j) = costMat(j, i) = dist;
    }
  }
}

void MTSPModel::ga_init_population(int groupSize_, int generations_, double crossoverProb_) {
  groupSize = groupSize_;
  generations = generations_;
  crossoverProb = crossoverProb_;

  minCost = 999;
  optimum_first = std::vector<int>(numCity), optimum_second = std::vector<int>(numSalemen);
  selection_pool = std::vector<int>(groupSize*crossoverProb), replacement_pool = std::vector<int>(groupSize*crossoverProb);

  population = std::vector<Indivisual>(groupSize, Indivisual(numCity, numSalemen));
  for (int i = 0; i < groupSize; ++i) {
    // 生成访问顺序
    population[i].firstGene = get_random_permutation(numCity);
    std::vector<int> index(numCity);
    for (int j=0; j<numCity; ++j) {
      index[population[i].firstGene[j]] = j;
    }
    population[i].index = index;
    // 生成分配序列
    population[i].secondGene = get_sequence_with_sum(numCity, numSalemen);
  }
}

double MTSPModel::ga_evaluate_fitness(Indivisual& ind) {
  int begNodeIdx = 0, salemenIdx = 0;
  for (int j=0; j<numCity; ++j) {
    // 当前城市是商人出发后的第一个城市
    if (j == begNodeIdx) {
      ind.fitness += (city[ind.firstGene[j]] - depot[salemenIdx]).norm();
      // 跳过访问0城市的商人
      do {
      // 下个商人的第一个城市的索引
        begNodeIdx += ind.secondGene[salemenIdx];
        salemenIdx++;
      } while (ind.secondGene[salemenIdx] == 0);
    }
    else {
      ind.fitness += costMat(ind.firstGene[j-1], ind.firstGene[j]);
    }
  }
  return ind.fitness;
}

void MTSPModel::ga_evaluate_fitness() {
  // 评估个体适应性
  for (int i=0; i<groupSize; ++i) {
    ga_evaluate_fitness(population[i]);
  }
  // 个体适应性统计
  double tmpCost;
  sumCost = 0;
  for (int i=0; i<groupSize; ++i) {
    tmpCost = population[i].fitness;
    maxCost = std::max(maxCost, tmpCost);
    sumCost += tmpCost;
    if (tmpCost < minCost) {
      minCost = tmpCost;
      optimumInd = population[i];
      optimum_first = population[i].firstGene;
      optimum_second = population[i].secondGene;
    }
  }
}

void MTSPModel::ga_update_roulette_wheel() {
  sumCost = 0;
  for (int i=0; i<groupSize; ++i) {
    sumCost += population[i].fitness;
  }
  for (int i=0; i<groupSize; ++i) {
    population[i].wheelPartion = population[i].fitness/sumCost;
  }
}

int MTSPModel::ga_roll_wheel() {
  double prob = get_uniform_double();
  // std::cout << prob << std::endl;
  double probBase = 0.0;
  int choosen = -1;
  for (int i=0; i<groupSize; ++i) {
    probBase += population[i].wheelPartion;
    if (prob < probBase) {
      choosen = i;
      break;
    }
  }
  return choosen;
}

void MTSPModel::ga_crossover() {
  // 在配对池中的个体两两配对
  int poolSize = groupSize * crossoverProb;
  for (int i=0; i<poolSize/2; ++i) {
  }

}

Indivisual MTSPModel::ga_crossover_tcx(Indivisual mon, Indivisual dad) {
  // Do two-part chromosome crossover
  std::vector<std::vector<int>> savedGene(numSalemen);
  std::vector<int> restGene, secondGene = mon.secondGene;

  // 从母亲选择需要保留和重新分配的基因
  int baseCity = 0;
  for (int i=0; i<numSalemen; ++i) {
    int assignedCity = mon.secondGene[i], beg = 0, end = 0;
    // 随机选择两个断点，注意是从1开始的
    std::vector<int> segment = get_uniform(assignedCity, 2);
    if (segment[0] < segment[1]) {
      beg = segment[0] - 1, end = segment[1] - 1;
    } else {
      beg = segment[1] - 1, end = segment[0] - 1;
    }
    if (assignedCity) secondGene[i] = end - beg + 1;

    // 分别记录需要保留和重新分配的基因段
    std::vector<int> saved;
    for (int j=0; j<assignedCity; ++j) {
      if (j < beg || j > end) {
        restGene.push_back(mon.firstGene[j + baseCity]);
      } else {
        saved.push_back(mon.firstGene[j + baseCity]);
      }
    }
    savedGene[i] = saved;
    baseCity += assignedCity;
  } // for (int i=0; i<numSalemen; ++i)

  // 从父亲获取 restGene 的排序
  if (dad.index.empty()) {printf("Dad dont init index vector\n");}
  sort(restGene.begin(), restGene.end(), [dad](const int &x, const int &y) {
      return dad.index[x] < dad.index[y];
  });

  // 从父亲获取 restGene 的分配
  int idxSalemen = 0;
  baseCity = 0;
  for (int i=0; i<restGene.size(); ++i) {
    int idxCurGene = dad.index[restGene[i]];
    while (idxCurGene > baseCity + dad.secondGene[idxSalemen]) {
      baseCity += dad.secondGene[idxSalemen];
      idxSalemen++;
    }
    savedGene[idxSalemen].push_back(restGene[i]);
    secondGene[idxSalemen]++;
  }

  // Rearrange first gene
  Indivisual sister;
  sister.secondGene = secondGene;
  for (int i=0; i<numSalemen; ++i) {
    for (int j=0; j<savedGene[i].size(); ++j) {
      sister.firstGene.push_back(savedGene[i][j]);
    }
  }
  std::vector<int> index(numCity);
  for (int j=0; j<numCity; ++j) {
    index[sister.firstGene[j]] = j;
  }
  sister.index = index;
  return sister;
}

