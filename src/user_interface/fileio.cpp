#include "fileio.h"

void read_vec_from_file(const std::string& fname, std::vector<std::vector<double>>& data) {
  std::ifstream input(fname);
  if (!input.is_open()) {
    printf("Error: failed to open file.\n");
  }

  data.clear();
  size_t row = 0;
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
    ++row;
  }
}
