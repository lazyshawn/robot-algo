#pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>

/* 
* @brief : 以向量的形式从文件读取数据
* @param : fname - 文件名称
* @return: data - 二维数组
*/
void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data);

/* 
* @brief : 以矩阵的形式从文件读取数据
* @param : fname - 文件名称
* @return: data - Eigen::MatrixXd 矩阵
* @ref   : https://stackoverflow.com/a/39146048
*/
void read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat);

