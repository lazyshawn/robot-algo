#include "geometry/kd_tree.h"

KdNode::KdNode(const Eigen::Vector3d& point_) {
  point = point_;
}

KdNode::KdNode(const Eigen::Vector3d& point_, double distance_) {
  point = point_;
  distance = distance_;
}

void KdTree::build_tree() {
  // 计算主轴方向
  principal_component_analysis(pointSet, axis);
  // 递归分配子树
  root = split_point_set(0, pointSet.size()-1, 0);
}

bool KdTree::compare_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d direction) {
  return p1.dot(direction) < p2.dot(direction);
}

std::unique_ptr<KdNode> KdTree::split_point_set(int start, int end, int depth) {
  // 递归的终止条件
  if (start > end) {
    return nullptr;
  }

  // 沿主轴方向排序
  Eigen::Vector3d direction = axis[depth % axis.size()];
  std::sort(pointSet.begin()+start, pointSet.begin()+end+1,
    [direction](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
      return p1.dot(direction) < p2.dot(direction);
    }
  );

  // 当前方向的中间点
  int mid = (start + end) / 2;
  std::unique_ptr<KdNode> curNode = std::make_unique<KdNode>(pointSet[mid]);

  // 递归
  curNode->left = split_point_set(start, mid - 1, depth + 1);
  curNode->right = split_point_set(mid + 1, end, depth + 1);

  return curNode;
}

void KdTree::search_knn(const Eigen::Vector3d& target, int k, std::vector<Eigen::Vector3d>& neighbors) {
  std::priority_queue<KdNode> pq;
  double distThreshold = std::numeric_limits<double>::infinity();

  std::function<void(KdNode*)> search = [&](KdNode* node) {
    if (node == nullptr)  return;

    double distance = (target - node->point).squaredNorm();
    if (pq.size() < k || distance < distThreshold) {
      pq.push(KdNode(node->point, distance));
      if (pq.size() > k) {
        pq.pop();
        distThreshold = pq.top().distance;
      }
    }

    int axisIdx = node->depth % target.size();
    double diff = target[axisIdx] - node->point[axisIdx];
    // 沿当前节点的分割面法向量，目标点在负方向
    if (diff < 0) {
      // 目标点在负方向，搜索左子树
      search(node->left.get());
      // 左子树内k个最近点中，最大距离大于目标点到当前分割平面的距离，搜索右子树
      if (diff*diff < distThreshold || pq.size() < k) {
        search(node->right.get());
      }
    }
    // 沿当前节点的分割面法向量，目标点在正方向
    else {
      search(node->right.get());
      if (diff*diff < distThreshold || pq.size() < k) {
        search(node->left.get());
      }
    }
  };

  search(root.get());

  while(!pq.empty()) {
    neighbors.emplace_back(pq.top().point);
    pq.pop();
  }
}

