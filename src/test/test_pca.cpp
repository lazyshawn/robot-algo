#include "geometry/algebra.h"
#include "user_interface/user_interface.h"

int main(int argc, char** argv) {
  int numPoints = 10;
  Eigen::MatrixXd pntMat = get_random_matrix(3, numPoints, 0, 10);
  std::vector<Eigen::Vector3d> points(numPoints);
  for (int i=0; i<numPoints; ++i) {
    points[i] = pntMat.col(i);
  }

  std::vector<Eigen::Vector3d> axis;
  Eigen::Vector3d com;
  Eigen::Vector3d singularValue = principal_component_analysis(points, axis, com);
  std::cout << "===> Singular Value:\n" << singularValue.transpose() << std::endl;

  std::cout << "\n===> axis:" << std::endl;
  for (int i=0; i<axis.size(); ++i) {
    std::cout << axis[i].transpose() << std::endl;
  }

  // 记录控制点
  std::ofstream pntFile("build/data/pca_points", std::ios::trunc);
  for (int i=0; i<static_cast<int>(numPoints); ++i) {
    pntFile << points[i].transpose() << std::endl;
  }
  std::ofstream pcaFile("build/data/pca_output", std::ios::trunc);
  pcaFile << com.transpose() << std::endl;
  for (int i=0; i<static_cast<int>(axis.size()); ++i) {
    pcaFile << axis[i].transpose() << std::endl;
  }
  return 0;
}

