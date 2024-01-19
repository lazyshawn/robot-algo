#include "user_interface/fileio.h"

uint8_t read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data, int rows) {
  std::ifstream input(fname);
  if (!input.is_open()) {
    printf("Error: failed to open file.\n");
    return 0x01;
  }

  data.clear();
  std::string line;
  while ((rows > 0 || rows + 1 == 0) && std::getline(input, line)) {
    std::vector<double> curData;
    std::stringstream lineStream(line);
    double num;

    // @see    https://stackoverflow.com/a/24520662
    while (lineStream >> num || !lineStream.eof()) {
      // 数据读取错误
      if (lineStream.fail()) {
        return 0x02;
      }
      curData.push_back(num);
    }
    data.emplace_back(curData);

    if (rows > 0) rows--;
  }
  return 0x00;
}

uint8_t read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat, int rows) {
  std::vector<std::vector<double>> data;
  if (uint8_t readFlag = read_vec_from_file(fname, data, rows); readFlag > 0) {
    return readFlag;
  }
  size_t dataRows = data.size(), dataCols = data[0].size();

  std::vector<double> values;
  values.reserve(dataRows*dataCols);
  for (size_t i=0; i<dataRows; ++i) {
    // 第 i 行元素个数错误
    if (dataCols != data[i].size()) {
      return 0x03;
    }
    for (size_t j=0; j<dataCols; ++j) {
      values.emplace_back(data[i][j]);
    }
  }
  mat = std::move(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(values.data(),dataRows,dataCols));
  return 0x00;
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

