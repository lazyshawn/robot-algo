#include "path_planning/probabilistic_roadmap.h"

PRM_Scene::PRM_Scene(const std::vector<double>& sceneConfig) {
  length = sceneConfig[0];
  width = sceneConfig[1];
  maxRadius = sceneConfig[2];

  obsPos.reserve(sceneConfig[3]);
  radius.reserve(sceneConfig[3]);
}

void PRM_Scene::load_scene(const std::vector<double>& obstacle) {
  int numObs = obstacle.size() / 3;
  for (size_t i=0; i<numObs; ++i) {
    obsPos.emplace_back(length * obstacle[i], width * obstacle[numObs + i]);
    radius.emplace_back(maxRadius * obstacle[2*numObs + i]);
  }
}

bool PRM_Scene::check_config_collision(const config_type& config) {
  for (size_t i=0; i<obsPos.size(); ++i) {
    if ((obsPos[i] - config).norm() < radius[i]) {
      return true;
    }
  }
  return false;
}

bool PRM_Scene::check_config_visibility(const config_type& configA, const config_type& configB, double resolution) {
  // 离散点数量
  int num = std::ceil((configA - configB).norm() / resolution);
  double det = 1.0 / num;
  // 判断可见性，等价于判断离散点是否在障碍物内
  for (size_t i=1; i<num; ++i) {
    double lamb = det * i;
    config_type tmp = lamb * configA + (1 - lamb) * configB;
    if (check_config_collision(tmp)) return false;
  }
  return true;
}

config_type PRM_Scene::sample_collision_free_config() {
  config_type sample;
  while (true) {
    std::vector<double> randNums = get_uniform_double(2);
    sample = config_type{randNums[0]*length, randNums[1]*width};
    // 生成无碰撞的节点
    if (!check_config_collision(sample)) {break;}
  }
  return sample;
}

PRM_Node::PRM_Node(config_type config_, NodeType nodeType, size_t idx_) {
  config = config_;
  type = nodeType;
  idx = idx_;
}

void PRM_Graph::add_node(const config_type& config, PRM_Node::NodeType nodeType) {
  node.emplace_back(PRM_Node(config, nodeType, nodeIdx++));
}

PRM_Planner::PRM_Planner(const std::vector<double>& obstacle) {
  scene = std::make_unique<PRM_Scene>(obstacle);
  roadmap = std::make_unique<PRM_Graph>();
}

void PRM_Planner::construct_visib_roadmap(int maxTry) {
  // 开始采样节点
  size_t numTry = 0;
  while (numTry < maxTry) {
    // 采样
    config_type sample = scene->sample_collision_free_config();

    // 上一个可见节点
    std::vector<size_t> gVis;
    // 上一个可见的连同分量
    std::list<std::list<size_t>>::iterator compIte;
    bool findConnection = false;
    // 开始遍历连通分量
    for (std::list<std::list<size_t>>::iterator ite=guardList.begin(); ite!=guardList.end(); ++ite) {
      std::list<size_t>& components = *ite;
      bool findVis = false;

      // 开始遍历节点
      for (auto& node : components) {
        // 判断可见性
        if (scene->check_config_visibility(roadmap->node[node].config, sample)) {
          findVis = true;
          if (gVis.empty()) {
            // 记录可以看见采样点的 guard 节点和连同分量
            gVis.emplace_back(node);
            compIte = ite;
          } else {
            findConnection = true;
            // 采样点连接了两个 guard 节点, 属于 connection 节点
            add_connection(sample);
            // 合并连通分量
            merge_connected_component(compIte, ite);
            // 记录新生成的边
            add_edge(gVis[0], roadmap->node.size()-1);
            add_edge(node, roadmap->node.size()-1);
          }
        } // for (node)
        // 找到可见的 guard 节点，跳出当前连同分量
        if (findVis) break;
      }
      // 找到 connection 节点，结束循环，开始找下一个节点
      if (findConnection) break;
    } // for (components)

    // 采样点是 guard 节点
    if (gVis.empty()) {
      std::cout << "Get new guard: " << sample.transpose() << std::endl;
      add_guard(sample);
      numTry = 0;
    } else {
      numTry++;
    }
  }
}

void PRM_Planner::merge_connected_component(std::list<std::list<size_t>>::iterator& compA, std::list<std::list<size_t>>::iterator& compB) {
  (*compA).splice((*compA).end(), (*compB));
  guardList.erase(compB);
}

void PRM_Planner::add_connection(const config_type& config) {
  roadmap->add_node(config, PRM_Node::NodeType::CONNECTION);
  // 记录新生成的 connection 节点序号
  connection.emplace_back(roadmap->node.size() - 1);
}

void PRM_Planner::add_guard(const config_type& config) {
  roadmap->add_node(config, PRM_Node::NodeType::GUARD);
  // 新增连同分量序列
  guardList.emplace_back(std::list<size_t>({roadmap->node.size() - 1}));
}

void PRM_Planner::add_edge(size_t nodeA, size_t nodeB) {
  edge.emplace_back(std::array<size_t,2>({nodeA, nodeB}));
}

