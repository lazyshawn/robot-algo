#pragma  once

#include <eigen3/Eigen/Dense>
// 生成随机数
#include <chrono>
#include <random>

Eigen::MatrixXd get_random_matrix(int raw, int col, double mean = 0.0, double std = 1.0);

/* 
* @brief : 生成 0 到 1 的均匀分布随机数序列
* @param : size - 生成的序列长度
* @return: 
*/
std::vector<double> get_uniform_double(int size=10);

