
#include "kd_tree.h"
#include "user_interface.h"

int main(int argc, char** argv) {
  int numPoints = 10;
  Eigen::MatrixXd pntMat = get_random_matrix(3, numPoints, 0, 10);
  std::cout << pntMat << std::endl;
  std::vector<Eigen::Vector3d> points(numPoints);
  for (int i=0; i<numPoints; ++i) {
    points[i] = pntMat.col(i);
  }

  std::cout << "===> Kd-tree:" << std::endl;
  Eigen::Vector3d x;
  kdtree::Node node(x);
  std::cout << node.dim << std::endl;
  return 0;
}

