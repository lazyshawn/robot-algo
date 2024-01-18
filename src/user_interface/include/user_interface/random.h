/**
* @file   random.h
* @brief  随机数操作
*/
#pragma  once

#include <eigen3/Eigen/Dense>
// 生成随机数
#include <chrono>
#include <random>

/**
* @brief  生成元素为正态分布的矩阵
* @param  raw  矩阵行数
* @param  col  矩阵列数
* @param  mean 正态分布均值
* @param  std  正态分布标准差
* @return 随机数序列
*/
Eigen::MatrixXd get_random_matrix(int raw, int col, double mean = 0.0, double std = 1.0);

/**
* @brief  生成 0 到 1 的均匀分布随机数序列
* @param  size 生成的序列长度
* @return 随机数序列
*/
std::vector<double> get_uniform_double(int size=10);

