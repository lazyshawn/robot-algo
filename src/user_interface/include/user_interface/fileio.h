#pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>

/* 
* @brief : 以向量的形式从文件读取数据
* @param : fname - 文件名称
* @return: data - 二维数组
*/
void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data, int rows = -1);

/* 
* @brief : 以矩阵的形式从文件读取数据
* @param : fname - 文件名称
* @return: data - Eigen::MatrixXd 矩阵
* @ref   : https://stackoverflow.com/a/39146048
*/
void read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat, int rows = -1);

/* 
* @brief : 检查 json 数据中是否存在给定的字段
* @param : data - json 数据
* @param : field - 待检查的字段
* @param : numItem - 字段内期望的元素个数 - 0
* @return: 
*/
std::optional<nlohmann::json> get_json_field(nlohmann::json data, std::string field, size_t numItem = 0);

