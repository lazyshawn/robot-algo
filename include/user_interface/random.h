#pragma  once

#include <eigen3/Eigen/Dense>
// 生成随机数
#include <chrono>
#include <random>

Eigen::MatrixXd get_random_matrix(int raw, int col, double mean = 0.0, double std = 1.0);

