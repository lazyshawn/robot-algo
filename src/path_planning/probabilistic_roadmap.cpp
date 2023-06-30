#include "probabilistic_roadmap.h"

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

bool PRM_Scene::check_config_collision(config_type config) {
  for (size_t i=0; i<obsPos.size(); ++i) {
    if ((obsPos[i] - config).norm() < radius[i]) {
      return true;
    }
  }
  return false;
}

bool PRM_Scene::check_config_visibility(config_type configA, config_type configB) {
  // 离散点数量
  int segment = 100;
  double det = 1.0 / segment;
  // 判断可见性，等价于判断离散点是否在障碍物内
  for (size_t i=1; i<segment; ++i) {
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

PRM_Planner::PRM_Planner(const std::vector<double>& obstacle) {
  scene = std::make_unique<PRM_Scene>(obstacle);
  roadmap = std::make_unique<PRM_Graph>();
}

void PRM_Planner::construct_visib_roadmap(int maxTry) {
  // 节点
  std::vector<config_type> connection;
  std::list<std::list<config_type>> guardList;

  // 开始采样节点
  size_t numTry = 0;
  while (numTry < maxTry) {
    // 采样
    config_type sample = scene->sample_collision_free_config();

    // 上一个可见节点
    std::vector<config_type> gVis;
    // 上一个可见的连同分量
    std::list<std::list<config_type>>::iterator compIte;
    bool findConnection = false;
    // 开始遍历连通分量
    for (std::list<std::list<config_type>>::iterator ite=guardList.begin(); ite!=guardList.end(); ++ite) {
      std::list<config_type>& components = *ite;
      bool findVis = false;

      // 开始遍历节点
      for (auto& node : components) {
        // 判断可见性
        if (scene->check_config_visibility(node, sample)) {
          findVis = true;
          if (gVis.empty()) {
            // 记录可以看见采样点的 guard 节点
            gVis.emplace_back(node);
            compIte = ite;
          } else {
            findConnection = true;
            // 采样点连接了两个 guard 节点, 属于 connection 节点
            connection.emplace_back(sample);
            // 合并连通分量
            (*compIte).splice((*compIte).end(), (*ite));
            guardList.erase(ite);
            // 记录新生成的边
            // edges.emplace_back(gVis[0]);
            // edges.emplace_back(sample);
            // edges.emplace_back(node);
            PRM_Node node(sample, PRM_NodeType::CONNECTION);
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
      guardList.emplace_back(std::list<config_type>({sample}));
      numTry = 0;
    } else {
      numTry++;
    }
  }
}

PRM_Node::PRM_Node(config_type config_, PRM_NodeType nodeType) {
  config = config_;
  type = nodeType;
}
