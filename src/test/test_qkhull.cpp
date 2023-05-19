#include "convex_decomposition/convex_decomposition.h"
#include "user_interface/user_interface.h"

#include <random>
#include <ctime>

int main(int argc, char** argv) {
  Timer timer;
  std::vector<Eigen::Vector3d> set;

  std::default_random_engine generator(time(0));
  std::normal_distribution<double> distribution(10,3);
  auto normal = [&] () {return distribution(generator);};

  std::ofstream pointFile("build/data/qkhull_points", std::ios::trunc);
 
  // ramdom matrix
  Eigen::MatrixXd randomMat = Eigen::MatrixXd::NullaryExpr(3,10,normal);
  for (int i=0; i<randomMat.cols(); ++i) {
    set.emplace_back(randomMat.col(i));
    pointFile << set[i].transpose() << std::endl;
  }
  std::cout << randomMat << "\n" << std::endl;

  qkhull::FaceList cvxHull;
  timer.start();
  qkhull::quickhull(set, cvxHull);
  std::chrono::duration<double, std::milli> usageTime = timer.time_since_last();
  std::cout << "Total time: " << usageTime.count() << "ms" << std::endl;

  // 记录凸包表面
  std::ofstream hullFile("build/data/qkhull_hull", std::ios::trunc);
  for (qkhull::FaceIterator iteF=cvxHull.begin(); iteF!=cvxHull.end(); ++iteF) {
    for (int i=0; i<3; ++i) {
      hullFile << (*iteF)->vertex[i]->point.transpose() << std::endl;
    }
  }
  return 0;
}

