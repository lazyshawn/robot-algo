#include "collision_detection/gjk.h"
#include "user_interface/fileio.h"
#include <iostream>

#include <random>
#include <ctime>
 

int main(int argc, char** argv) {
  std::default_random_engine generator(time(0));
  std::normal_distribution<double> distribution(10,3);
  auto normal = [&] () {return distribution(generator);};

  std::ofstream ofile("build/data/simplex_points", std::ios::trunc);
 
  // ramdom matrix
  // Eigen::MatrixXd randomMat = Eigen::MatrixXd::Random(3,6);
  Eigen::MatrixXd randomMat = Eigen::MatrixXd::NullaryExpr(3,8,normal);
  // convex set
  int dim = randomMat.cols()/2;
  std::vector<Eigen::Vector3d> setA = std::vector<Eigen::Vector3d>(dim);
  std::vector<Eigen::Vector3d> setB(setA);
  for (int i=0; i<randomMat.cols()/2; ++i) {
    setA[i] = randomMat.col(i);
    setB[i] = randomMat.col(i+randomMat.cols()/2);
  }

  // std::cout << "Set A:" << std::endl;
  for (int i=0; i<setA.size(); ++i) {
    // std::cout << setA[i].transpose() << std::endl;
    ofile << setA[i].transpose() << std::endl;
  }
  // std::cout << "Set B:" << std::endl;
  for (int i=0; i<setB.size(); ++i) {
    // std::cout << setB[i].transpose() << std::endl;
    ofile << setB[i].transpose() << std::endl;
  }

  Eigen::Vector3d dir = {1, 0, 0};
  bool collision = gjk_collision_detection(setA, setB);
  if (collision) {
    std::cout << "collision detected!" << std::endl;
  } else {
    std::cout << "collision free." << std::endl;
    std::cout << gjk_closest_distance(setA, setB) << std::endl;
  }
  return 0;
}
