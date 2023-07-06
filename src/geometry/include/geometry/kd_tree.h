#pragma once

#include "algebra.h"

#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>

typedef std::list<Eigen::Vector3d>::iterator VecListIte;

class KdNode {
public:
  using KdNodePtr = std::shared_ptr<KdNode>;
  Eigen::Vector3d point;
  int depth;
  VecListIte begIte, endIte;
  std::unique_ptr<KdNode> left, right, parent;
  double distance = 0.0;

public:
  KdNode(const Eigen::Vector3d& point_);
  KdNode(const Eigen::Vector3d& point_, double distance_);

  bool operator< (const KdNode& other) const {
    return distance < other.distance;
  }

private:
};

class KdTree {
public:
  // 储存的根节点
  std::unique_ptr<KdNode> root;
  Eigen::Vector3d point;
  std::vector<Eigen::Vector3d> pointSet;
  std::vector<Eigen::Vector3d> axis;
  // 排序时考虑的最大节点数
  int maxSortNum;

public:
  KdTree() : root(nullptr) {};

  template <typename Container>
  void load_point_set(const Container& points);

  void build_tree();

  bool compare_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d direction);

  std::unique_ptr<KdNode> split_point_set(int start, int end, int depth);

  void search_knn(const Eigen::Vector3d& target, int k, std::vector<Eigen::Vector3d>& neighbors);
};

/* *****************************************************************************
 * Template
 * ************************************************************************** */
template <typename Container>
void KdTree::load_point_set(const Container& points) {
  int num = points.size();
  pointSet = std::vector<Eigen::Vector3d>(num);

  auto pntIte = points.begin();
  for (int i = 0; i < num; ++i) {
    pointSet[i] = *pntIte;
    pntIte++;
  }
}

