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
* @see    https://stackoverflow.com/a/24520662
* @return 读取状态
*       0 读取正常
*       1 文件打开失败，文件不存在等错误
*       2 数据读取失败，数据类型错误
*/
uint8_t read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data, int rows = -1);

/**
* @brief  以矩阵的形式从文件读取数据
* @param  fname     文件名称
* @param  [out] mat 矩阵
* @param  rows      读取行数, -1 表示读取全部
* @see    https://stackoverflow.com/a/39146048
* @return 读取状态
*       0 读取正常
*       1 文件打开失败，文件不存在等错误
*       2 数据读取失败，数据类型、存储结构等错误
*       3 数据结构错误，行元素数量不相等
*/
uint8_t read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat, int rows = -1);

/**
* @brief  检查 json 数据中是否存在给定的字段
* @param  data    json 数据
* @param  field   待检查的字段
* @param  numItem 字段内期望的元素个数
* @return json 字段
*/
std::optional<nlohmann::json> get_json_field(nlohmann::json data, std::string field, size_t numItem = 0);

