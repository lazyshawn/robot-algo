#pragma once

#include <list>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "user_interface.h"

typedef Eigen::Vector2d config_type;

// enum class PRM_NodeType {CONNECTION, GUARD};

class PRM_Scene {
public:
  double length, width, maxRadius;
public:
  PRM_Scene(const std::vector<double>& sceneConfig);
  // 障碍物模型
  std::vector<config_type> obsPos;
  std::vector<double> radius;

public:
  void load_scene(const std::vector<double>& obstacle);
  // 检测状态是否无碰撞
  bool check_config_collision(const config_type& config);
  // 不同状态之间的可见性
  bool check_config_visibility(const config_type& configA, const config_type& configB, double resolution=1e-1);
  // 采样无碰撞点
  config_type sample_collision_free_config();
  // 检测可见性压缩
  // bool check_visibility_deformation(const std::list<config_type>& pathA, const std::list<config_type>& pathB);
  // 节点可见路径的连通性
  // bool roadmap_connected_from_point(const config_type& viewer);
};


class PRM_Node {
public:
  enum class NodeType {CONNECTION, GUARD};
  size_t idx;
  NodeType type;
  Eigen::Vector2d config;
public:
  PRM_Node(config_type config_, NodeType nodeType, size_t idx_);
};


class PRM_Graph {
public:
  std::vector<PRM_Node> node;
  std::vector<std::vector<int>> edge;
public:
  void add_node(const config_type& config, PRM_Node::NodeType nodeType);
private:
  size_t nodeIdx = 0;
};


class PRM_Planner {
public:
  std::unique_ptr<PRM_Scene> scene;
  std::unique_ptr<PRM_Graph> roadmap;
  std::vector<size_t> connection;
  std::list<std::list<size_t>> guardList;
  std::vector<std::array<size_t,2>> edge;
public:
  PRM_Planner(const std::vector<double>& obstacle);
  void construct_visib_roadmap(int maxTry = 1e4);
  void merge_connected_component(std::list<std::list<size_t>>::iterator& compA, std::list<std::list<size_t>>::iterator& compB);
  void add_connection(const config_type& config);
  void add_guard(const config_type& config);
  void add_edge(size_t nodeA, size_t nodeB);
};

