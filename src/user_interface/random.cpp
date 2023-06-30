// user_interface
#include "random.h"

Eigen::MatrixXd get_random_matrix(int raw, int col, double mean, double std) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(mean,std);
  auto normal = [&] () {return distribution(generator);};
  // ramdom matrix
  Eigen::MatrixXd randomMat = Eigen::MatrixXd::NullaryExpr(raw,col,normal);

  return randomMat;
}

std::vector<double> get_uniform_double(int size) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> uniform(0, 1);

  std::vector<double> randomVector(size);
  for (int i=0; i<size; ++i) {
    randomVector[i] = uniform(generator);
  }
  return randomVector;
}

