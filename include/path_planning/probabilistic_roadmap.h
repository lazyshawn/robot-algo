#pragma once

#include <list>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "user_interface.h"

typedef Eigen::Vector2d config_type;

enum class PRM_NodeType {CONNECTION, GUARD};

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
  bool check_config_collision(config_type config);
  bool check_config_visibility(config_type configA, config_type configB);
  config_type sample_collision_free_config();
};


class PRM_Node {
public:
  int idx;
  PRM_NodeType type;
  Eigen::Vector2d config;
public:
  PRM_Node(config_type config_, PRM_NodeType nodeType);
};


class PRM_Graph {
public:
  std::vector<PRM_Node> node;
  std::vector<std::vector<int>> edge;
};


class PRM_Planner {
public:
  std::unique_ptr<PRM_Scene> scene;
  std::unique_ptr<PRM_Graph> roadmap;
public:
  PRM_Planner(const std::vector<double>& obstacle);
  void construct_visib_roadmap(int maxTry = 1e4);
};

