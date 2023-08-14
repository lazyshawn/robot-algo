#include "user_interface/fileio.h"

void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data, int rows) {
  std::ifstream input(fname);
  if (!input.is_open()) {
    printf("Error: failed to open file.\n");
  }

  data.clear();
  std::string line;
  // while (std::getline(input, line)) {
  while ((rows > 0 || rows + 1 == 0) && std::getline(input, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> curData;
    curData.reserve(3);
    while (std::getline(lineStream, cell, ' ')) {
      curData.emplace_back(std::stod(cell));
    }
    data.emplace_back(curData);

    if (rows > 0) rows--;
  }
}

void read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat, int rows) {
  std::vector<std::vector<double>> data;
  read_vec_from_file(fname, data, rows);
  size_t dataRows = data.size(), dataCols = data[0].size();

  std::vector<double> values;
  values.reserve(dataRows*dataCols);
  for (size_t i=0; i<dataRows; ++i) {
    for (size_t j=0; j<dataCols; ++j) {
      values.emplace_back(data[i][j]);
    }
  }
  mat = std::move(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(values.data(),dataRows,dataCols));
}

std::optional<nlohmann::json> get_json_field(nlohmann::json data, std::string field, size_t numItem) {
  if(data.find(field) == data.end()) {
    // 字段不存在
    std::cout << "Error: field [" << field << "] does not exist." << std::endl;
    return std::nullopt;
  } else if (numItem > 0) {
    // 需要检查元素个数
    size_t num = data[field].size();
    if(num < numItem) {
      // 元素个数过少
      std::cout << "Error: field [" << field << "] require " << numItem << ", but got " << num << std::endl;
      return std::nullopt;
    } else if(num > numItem) {
      // 元素个数过多
      std::cout << "Warnning: field [" << field << "] require " << numItem << ", but got " << num << std::endl;
    }
  }
  return std::make_optional(data[field]);
}

