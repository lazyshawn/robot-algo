
#include "data_processing/filter.h"
#include "user_interface/user_interface.h"

#include <iostream>

const std::string savePath = "build/data/data_processing/";
void test_butterworth();

int main(int argc, char** argv) {

  return 0;
}

void test_butterworth() {
  Eigen::MatrixXd mat;
  read_eigen_from_file("data/current.txt", mat);
  // std::cout << mat.rows() << ", " << mat.cols() << std::endl;

  std::vector<float> raw(mat.rows());
  // std::cout << mat << std::endl;
  for (size_t i=0; i<mat.rows(); ++i) {
    raw[i] = mat(i);
  }

  // 采样频率
  float sampleFreq = 100;
  // for (size_t i=0; i<raw.size(); ++i) {
  //   float t = (float)i / sampleFreq;
  //   float tmp = fabs(6*sin(2*M_PI*3*t) + 6*sin(2*M_PI*50*t) + 6*sin(2*M_PI*500*t));
  //   raw[i] = tmp;
  // }

  // 滤波
  float wc = 2.2 * 2*M_PI/sampleFreq;
  std::vector<float> out = butterworth_filter(raw, wc);
  // out = butterworth_filter(out, w);

  std::ofstream outRaw(savePath + "raw");
  for (size_t i=0; i<raw.size(); ++i) {
    outRaw << raw[i] << " " << out[i] << std::endl;
  }
}
