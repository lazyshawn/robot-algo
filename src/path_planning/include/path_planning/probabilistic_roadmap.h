/**
* @file   probabilistic_roadmap.h
* @brief  概率路线图
*/
#pragma once

#include <list>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "user_interface/user_interface.h"

/**
* @brief  定义物体状态的数据类型
*/
typedef Eigen::Vector2d config_type;

// enum class PRM_NodeType {CONNECTION, GUARD};

/**
* @brief  场景构建
*
* 自定义场景。有效范围，障碍物位姿及几何形状，障碍物距离函数，可见性检测等
*/
class PRM_Scene {
public:
  double length, width, maxRadius;
public:
  PRM_Scene(const std::vector<double>& sceneConfig);
  // 障碍物模型
  std::vector<config_type> obsPos;
  std::vector<double> radius;

public:
  /**
  * @brief  加载场景
  */
  void load_scene(const std::vector<double>& obstacle);
  /**
  * @brief  检测状态是否无碰撞
  */
  bool check_config_collision(const config_type& config);
  /**
  * @brief  不同状态之间的可见性
  */
  bool check_config_visibility(const config_type& configA, const config_type& configB, double resolution=1e-1);
  /**
  * @brief  采样无碰撞点
  */
  config_type sample_collision_free_config();
  // 检测可见性压缩
  // bool check_visibility_deformation(const std::list<config_type>& pathA, const std::list<config_type>& pathB);
  // 节点可见路径的连通性
  // bool roadmap_connected_from_point(const config_type& viewer);
};


/**
* @brief  路线图节点
*/
class PRM_Node {
public:
  enum class NodeType {CONNECTION, GUARD};
  size_t idx;
  NodeType type;
  Eigen::Vector2d config;
public:
  PRM_Node(config_type config_, NodeType nodeType, size_t idx_);
};


/**
* @brief  路线图对象
*/
class PRM_Graph {
public:
  std::vector<PRM_Node> node;
  std::vector<std::vector<int>> edge;
public:
  void add_node(const config_type& config, PRM_Node::NodeType nodeType);
private:
  size_t nodeIdx = 0;
};


/**
* @brief  路线图规划器
*/
class PRM_Planner {
public:
  std::unique_ptr<PRM_Scene> scene;
  std::unique_ptr<PRM_Graph> roadmap;
  std::vector<size_t> connection;
  std::list<std::list<size_t>> guardList;
  std::vector<std::array<size_t,2>> edge;
public:
  PRM_Planner(const std::vector<double>& obstacle);
  /**
  * @brief  构建可见性路线图
  *
  * 用最少的代表性节点覆盖无碰撞的状态空间
  * @param  maxTry 添加新节点的最大尝试次数
  */
  void construct_visib_roadmap(int maxTry = 1e4);
  /**
  * @brief  连接连通分量
  * @param  compA,compB 两个不同的连通分量
  */
  void merge_connected_component(std::list<std::list<size_t>>::iterator& compA, std::list<std::list<size_t>>::iterator& compB);
  /**
  * @brief  增加 connection 节点
  * @param  config 采样状态
  */
  void add_connection(const config_type& config);
  /**
  * @brief  增加 guard 节点
  * @param  config 采样状态
  */
  void add_guard(const config_type& config);
  /**
  * @brief  增加边
  * @param  nodeA,nodeB 边的两个端点
  */
  void add_edge(size_t nodeA, size_t nodeB);
};

