#include "user_interface.h"
#include "probabilistic_roadmap.h"

// 场地尺寸
const double length = 16, width = 9;
// 障碍物
const size_t numObs = 6;
const double maxRadius = 2;
// 规划器
PRM_Planner planner({length, width, maxRadius, numObs});

void test_check_visibility();

void record_obstacle();
void record_graph();

int main(int argc, char** argv) {
  std::cout << "===> Construct Scene:" << std::endl;
  // 获取随机数序列
  std::vector<double> randNums = get_uniform_double(3*numObs);
  // 根据随机数初始化障碍物
  planner.scene->load_scene(randNums);

  std::cout << "===> Construct visibility roadmap:" << std::endl;
  // test_check_visibility();
  planner.construct_visib_roadmap(1e4);

  record_obstacle();
  record_graph();
  return 0;
}

void test_check_visibility() {
  std::vector<double> randNums = get_uniform_double(2);
  Eigen::Vector2d g{randNums[0]*length, randNums[1]*width};
  planner.add_guard(g);

  size_t numTry = 0, maxTry = 1e5;
  while (numTry < maxTry) {
    // 采样无碰撞点
    Eigen::Vector2d sample = planner.scene->sample_collision_free_config();

    if (planner.scene->check_config_visibility(sample, g)) planner.add_connection(sample);
    numTry++;
  }
}

void record_obstacle() {
  std::ofstream output("build/data/prm_scene", std::ios::trunc);
  output << length << " " << width << std::endl;
  for (size_t i=0; i<planner.scene->obsPos.size(); ++i) {
    output << planner.scene->obsPos[i].transpose() << " " <<  planner.scene->radius[i] << std::endl;
  }
}

void record_graph() {
  std::ofstream outputConnection("build/data/prm_connection", std::ios::trunc);
  for (auto& nodeIdx : planner.connection) {
    outputConnection << planner.roadmap->node[nodeIdx].config.transpose() << std::endl;
  }

  std::ofstream outputGuard("build/data/prm_guard", std::ios::trunc);
  int numGuard = 0;
  for (auto& components : planner.guardList) {
    for (auto& nodeIdx : components) {
      outputGuard << planner.roadmap->node[nodeIdx].config.transpose() << std::endl;
      numGuard++;
    }
  }

  std::ofstream outputEdge("build/data/prm_edge", std::ios::trunc);
  for (auto& edge : planner.edge) {
    outputEdge << planner.roadmap->node[edge[0]].config.transpose() << std::endl;
    outputEdge << planner.roadmap->node[edge[1]].config.transpose()<< std::endl;
  }

  std::cout << "===> Summary:" << std::endl;
  std::cout << "Connections: " << planner.connection.size() << std::endl;
  std::cout << "Guards: " << numGuard << std::endl;
  std::cout << "components: " << planner.guardList.size() << std::endl;
  std::cout << "Edges: " << planner.edge.size() << std::endl;
}

