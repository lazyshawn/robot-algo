#pragma once

#include <fstream>
#include <sstream>
#include <vector>

/* 
* @brief : 以向量的形式从文件读取数据
* @param : fname - 文件名称
* @return: data - 数据
* @ref   : https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix/39146048#39146048
*/
void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data);

template<typename M>
M read_eigen_from_file(const std::string& fname) {
}
