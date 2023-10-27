/**
* @file   kd_tree.h
* @brief  Kd tree and KNN algorithm
*
* Kd 树与 KNN 算法
*/
#pragma once

#include "algebra.h"

#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>

typedef std::list<Eigen::Vector3d>::iterator VecListIte;

/**
* @brief Nodes of kd tree
*/
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

/**
* @brief Kd tree
*/
class KdTree {
public:
  //! 储存的根节点
  std::unique_ptr<KdNode> root;
  Eigen::Vector3d point;
  std::vector<Eigen::Vector3d> pointSet;
  std::vector<Eigen::Vector3d> axis;
  //! 排序时考虑的最大节点数
  int maxSortNum;

public:
  KdTree() : root(nullptr) {};

  /**
  * @brief  加载点集
  * @param  points 点集
  */
  template <typename Container>
  void load_point_set(const Container& points);

  /**
  * @brief  构建 kd tree
  */
  void build_tree();

  /**
  * @brief  比较两节点的距离
  * @param  p1,p2     待比较的两个节点
  * @param  direction 投影方向
  */
  bool compare_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d direction);

  /**
  * @brief  分配子树
  * @param  start,end 需要分配的节点始末索引
  * @param  depth     节点深度
  */
  std::unique_ptr<KdNode> split_point_set(int start, int end, int depth);

  /**
  * @brief  KNN 算法
  * @param  target          目标点
  * @param  k               搜索点数量
  * @param  [out] neighbors k 个临近点
  */
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

