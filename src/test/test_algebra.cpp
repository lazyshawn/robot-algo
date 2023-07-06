// geometry
#include "geometry/algebra.h"
#include <iostream>

int main(int argc, char** argv) {
  Eigen::Vector3d target{1,2,3}, norm{1,1,1}, viaPoint{0,0,0};
  std::cout << dist2plane(target, norm, viaPoint) << std::endl;
  return 0;
}
