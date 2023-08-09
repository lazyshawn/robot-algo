#include "user_interface/user_interface.h"
#include "geometry/transformation.h"

void test_ur_fk();

int main(int argc, char** argv) {
  test_ur_fk();

  return 0;
}


void test_ur_fk() {
  // UR5
  double l1 = 0.425, l2 = 0.392, h1 = 0.089, h2 = 0.095, w1 = 0.109, w2 = 0.082;

  Eigen::Isometry3d M(Eigen::Translation3d(Eigen::Vector3d(l1+l2, w1+w2, h1-h2)));
  M.linear() = Eigen::Matrix3d({{-1,0,0},{0,0,1},{0,1,0}});
  std::cout << "M0 = \n" << M.matrix() << std::endl;

  std::vector<Eigen::Vector<double,6>> twist(6);
  twist[0] = {0, 0, 0, 0, 0, 1};
  twist[1] = {-h1, 0, 0, 0, 1, 0};
  twist[2] = {-h1, 0, l1, 0, 1, 0};
  twist[3] = {-h1, 0, l1 + l2, 0, 1, 0};
  twist[4] = {-w1, l1 + l2, 0, 0, 0, -1};
  twist[5] = {h2 - h1, 0, l1 + l2, 0, 1, 0};

  Eigen::Vector<double,6> theta{0, -M_PI/2, 0, 0, M_PI/2, 0};
  std::cout << "theta = \n" << theta.transpose() << std::endl;

  Eigen::Isometry3d tran = M;
  for (size_t i=0; i<twist.size(); ++i) {
    int idx = twist.size() - i - 1;
    tran = lieSE3(twist[idx] * theta[idx]) * tran;
  }
  std::cout << "Tran = \n" << tran.matrix() << std::endl;
}
