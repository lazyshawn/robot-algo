
#include "data_processing/filter.h"
#include "user_interface/user_interface.h"

#include <iostream>

const std::string savePath = "build/data/data_processing/";

int main(int argc, char** argv) {
  std::cout << (19 & 1) << std::endl;
  // std::cout << "hello world" << std::endl;
  //
  // std::vector<float> raw(1500);
  // // 采样频率
  // float sampleFreq = 1000;
  // for (size_t i=0; i<raw.size(); ++i) {
  //   float t = (float)i / sampleFreq;
  //   float tmp = fabs(6*sin(2*M_PI*3*t) + 6*sin(2*M_PI*50*t) + 6*sin(2*M_PI*500*t));
  //   raw[i] = tmp;
  // }
  //
  // // 滤波
  // float w = 2*M_PI*3/sampleFreq;
  // std::vector<float> out = butterworth_filter(raw, w);
  // out = butterworth_filter(out, w);
  //
  // std::ofstream outRaw(savePath + "raw");
  // for (size_t i=0; i<raw.size(); ++i) {
  //   outRaw << raw[i] << " " << out[i] << std::endl;
  // }

  return 0;
}

