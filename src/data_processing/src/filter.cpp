#include "data_processing/filter.h"

std::vector<float> butterworth_filter(const std::vector<float>& raw, float wc) {
  // 过滤后的信号
  std::vector<float> out(raw.size());

  out[0] = raw[0];
  // 一阶 BF
  // for (size_t i=1; i<raw.size(); ++i) {
  //   out[i] = (wc*raw[i] + wc*raw[i-1] - (wc-1)*out[i-1]) / (wc + 1);
  // }

  // 二阶 BF
  for (size_t i=1; i<raw.size(); ++i) {
    float wc2 = wc * wc;
    float x1 = raw[i-1];
    float x2 = (int)i-2 < 0 ? raw[0] : raw[i-2];
    float y1 = out[i-1];
    float y2 = (int)i-2 < 0 ? out[0] : out[i-2];
    // out[i] = (wc2*raw[i] + 2*wc2*raw[i-1] + wc2*raw[i-2] - (-2+2*wc2)*out[i-1] - (1-1.4142*wc+wc2)*out[i-2]) / (1+1.4142*wc+wc2);
    out[i] = (wc2*raw[i] + 2*wc2*x1 + wc2*x2 - (-2+2*wc2)*y1 - (1-1.4142*wc+wc2)*y2) / (1+1.4142*wc+wc2);
  }

  return out;
}

void FFT() {

}
