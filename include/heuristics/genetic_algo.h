#pragma once

#include <algorithm>
#include <random>
#include <vector>
#include <chrono>


// 获取从 1~num 的随机排列
std::vector<int> get_random_permutation(int num);

// 获取从 1~num 的均匀分布随机数
int get_uniform(int num);

