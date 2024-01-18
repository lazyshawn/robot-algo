/**
* @file   fileio.h
* @brief  基础文件操作
*/
#pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>

/**
* @brief  以向量的形式从文件读取数据
* @param  fname      文件名称
* @param  [out] data 二维数组
* @param  rows       读取行数, -1 表示读取全部
*/
void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data, int rows = -1);

/**
* @brief  以矩阵的形式从文件读取数据
* @param  fname     文件名称
* @param  [out] mat 矩阵
* @param  rows      读取行数, -1 表示读取全部
* @see    https://stackoverflow.com/a/39146048
*/
void read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat, int rows = -1);

/**
* @brief  检查 json 数据中是否存在给定的字段
* @param  data    json 数据
* @param  field   待检查的字段
* @param  numItem 字段内期望的元素个数
* @return json 字段
*/
std::optional<nlohmann::json> get_json_field(nlohmann::json data, std::string field, size_t numItem = 0);

