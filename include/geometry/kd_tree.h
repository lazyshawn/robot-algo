#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>

namespace kdtree {
class Node {
public:
  Eigen::MatrixXd point;
  std::shared_ptr<Node> parent = nullptr, left = nullptr, right = nullptr;

public:
  const int dim;
  Node(Eigen::MatrixXd pnt);

private:
};

class Tree {
public:
  Node node;
  int axis, depth;
  std::shared_ptr<Tree> parent = nullptr, left = nullptr, right = nullptr;

public:
  Tree();
};

}; // namespace kdtree
