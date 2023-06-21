#include "fileio.h"

void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data) {
  std::ifstream input(fname);
  if (!input.is_open()) {
    printf("Error: failed to open file.\n");
  }

  data.clear();
  std::string line;
  while (std::getline(input, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> curData;
    curData.reserve(3);
    while (std::getline(lineStream, cell, ' ')) {
      curData.emplace_back(std::stod(cell));
    }
    data.emplace_back(curData);
  }
}

void read_eigen_from_file(const std::string &fname, Eigen::MatrixXd& mat) {
  std::vector<std::vector<double>> data;
  read_vec_from_file(fname,data);
  size_t rows = data.size(), cols = data[0].size();

  std::vector<double> values;
  values.reserve(rows*cols);
  for (size_t i=0; i<rows; ++i) {
    for (size_t j=0; j<cols; ++j) {
      values.emplace_back(data[i][j]);
    }
  }
  mat = std::move(Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(values.data(),rows,cols));
}

