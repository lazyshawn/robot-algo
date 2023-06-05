
#include "kd_tree.h"
#include "user_interface.h"

int main(int argc, char** argv) {
  // 生成随机测试数据
  int numPoints = 10;
  Eigen::MatrixXd pntMat = get_random_matrix(3, numPoints, 0, 10);
  std::cout << pntMat << std::endl;
  std::vector<Eigen::Vector3d> points(numPoints);
  for (int i=0; i<numPoints; ++i) {
    points[i] = (pntMat.col(i));
  }

  std::cout << "===> Kd-tree:" << std::endl;
  KdTree kdtree;
  kdtree.load_point_set(points);
  kdtree.build_tree();
  std::vector<Eigen::Vector3d> knn;
  Eigen::Vector3d target{0,10,0};
  kdtree.search_knn(target, 3, knn);

  std::cout << "\n===> KNN:" << std::endl;
  std::cout << "dist to all points:" << std::endl;
  for (auto& p : kdtree.pointSet) {
    std::cout << "dis = " << (p-target).norm() << ",  p: " << p.transpose() << std::endl;
  }
  std::cout << std::endl;
  std::cout << "dist to knn:" << std::endl;
  for (int i=0; i<knn.size(); ++i) {
    std::cout << "dis = " << (knn[i]-target).norm() << ",  p: " << knn[i].transpose() << std::endl;
  }

  // 记录控制点
  std::ofstream pntFile("build/data/kdtree_points", std::ios::trunc);
  for (auto& p : kdtree.pointSet) {
    pntFile << p.transpose() << std::endl;
  }
  std::ofstream pcaFile("build/data/kdtree_pca", std::ios::trunc);
  for (auto& axis : kdtree.axis) {
    pcaFile << axis.transpose() << std::endl;
  }
  std::ofstream outFile("build/data/kdtree_out", std::ios::trunc);
  outFile << target.transpose() << std::endl;
  for (auto& p : knn) {
    outFile << p.transpose() << std::endl;
  }
  return 0;
}

