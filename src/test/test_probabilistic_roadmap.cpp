#include "user_interface.h"
#include "probabilistic_roadmap.h"

// 场地尺寸
const double length = 16, width = 9;
// 障碍物
const size_t numObs = 6;
const double maxRadius = 2;

PRM_Planner planner({length, width, maxRadius, numObs});

std::vector<double> xCord(numObs), yCord(numObs), radius(numObs);
// 节点
std::vector<Eigen::Vector2d> connection;
std::list<std::list<Eigen::Vector2d>> guardList;
// 边
std::vector<Eigen::Vector2d> edges;

// 检测 config 是否碰撞
bool check_config_collision(Eigen::Vector2d config);
// 检测两节点是否可见
bool check_config_visibility(Eigen::Vector2d configA, Eigen::Vector2d configB);
// 构建可见性概率路线图
void construct_visib_roadmap(int maxTry = 1e4);
// 检测可见性压缩
bool check_visibility_deformation(const std::list<Eigen::Vector2d>& pathA, const std::list<Eigen::Vector2d>& pathB);
// 节点可见路径的连通性
bool roadmap_connected_from_point(Eigen::Vector2d viewer);

void test_check_visibility();

void record_obstacle();
void record_graph();

int main(int argc, char** argv) {
  std::cout << "===> Construct Scene:" << std::endl;
  // 获取随机数序列
  std::vector<double> randNums = get_uniform_double(3*numObs);
  // 根据随机数初始化障碍物
  for (size_t i=0; i<numObs; ++i) {
    xCord[i] = length * randNums[i];
    yCord[i] = width * randNums[numObs + i];
    radius[i] = maxRadius * randNums[2*numObs + i];
  }
  // planner.scene->load_scene(randNums);

  std::cout << "===> Sampling:" << std::endl;
  // planner.construct_visib_roadmap(1e4);
  construct_visib_roadmap(1e4);

  record_obstacle();
  record_graph();
  return 0;
}

void test_check_visibility() {
  std::vector<double> randNums = get_uniform_double(2);
  Eigen::Vector2d g{randNums[0]*length, randNums[1]*width};
  guardList.emplace_back(std::list<Eigen::Vector2d>({g}));

  size_t numTry = 0, maxTry = 1e5;
  while (numTry < maxTry) {
    // 采样
    randNums = get_uniform_double(2);
    Eigen::Vector2d sample{randNums[0]*length, randNums[1]*width};
    // 判断采样点本身是否无碰撞
    if (check_config_collision(sample)) continue;

    if (check_config_visibility(sample, g)) connection.emplace_back(sample);
    numTry++;
  }
}

void record_obstacle() {
  std::ofstream output("build/data/prm_scene", std::ios::trunc);
  output << length << " " << width << std::endl;
  for (size_t i=0; i<numObs; ++i) {
    output << xCord[i] << " " << yCord[i] << " " << radius[i] << std::endl;
  }
}

void record_graph() {
  std::ofstream outputConnection("build/data/prm_connection", std::ios::trunc);
  for (size_t i=0; i<connection.size(); ++i) {
    outputConnection << connection[i].transpose() << std::endl;
  }

  std::ofstream outputGuard("build/data/prm_guard", std::ios::trunc);
  int numGuard = 0;
  for (auto& components : guardList) {
    for (auto& node : components) {
      outputGuard << node.transpose() << std::endl;
      numGuard++;
    }
  }

  std::ofstream outputEdge("build/data/prm_edge", std::ios::trunc);
  for (size_t i=0; i<edges.size(); ++i) {
    outputEdge << edges[i].transpose() << std::endl;
  }

  std::cout << "===> Summary:" << std::endl;
  std::cout << "Connections: " << connection.size() << std::endl;
  std::cout << "Guards: " << numGuard << std::endl;
  std::cout << "components: " << guardList.size() << std::endl;
  std::cout << "Edges: " << edges.size()/3*2 << std::endl;
}



void construct_visib_roadmap(int maxTry) {
  size_t numTry = 0;
  // 采样点
  while (numTry < maxTry) {
    // 采样
    std::vector<double> randNums = get_uniform_double(2);
    Eigen::Vector2d sample{randNums[0]*length, randNums[1]*width};
    // 判断采样点本身是否无碰撞
    if (check_config_collision(sample)) continue;

    std::vector<Eigen::Vector2d> gVis;
    std::list<std::list<Eigen::Vector2d>>::iterator compIte;
    bool findConnection = false;
    // 开始遍历连通分量
    for (std::list<std::list<Eigen::Vector2d>>::iterator ite=guardList.begin(); ite!=guardList.end(); ++ite) {
      std::list<Eigen::Vector2d>& components = *ite;
      bool findVis = false;

      // 开始遍历节点
      for (auto& node : components) {
        // 判断可见性
        if (check_config_visibility(node, sample)) {
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
            edges.emplace_back(gVis[0]);
            edges.emplace_back(sample);
            edges.emplace_back(node);
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
      guardList.emplace_back(std::list<Eigen::Vector2d>({sample}));
      numTry = 0;
    } else {
      numTry++;
    }
  }
}

bool check_config_collision(Eigen::Vector2d config) {
  for (size_t i=0; i<numObs; ++i) {
    Eigen::Vector2d obsCent{xCord[i], yCord[i]};
    if ((obsCent - config).norm() < radius[i]) {
      return true;
    }
  }
  return false;
}

bool check_config_visibility(Eigen::Vector2d configA, Eigen::Vector2d configB) {
  // 离散点数量
  int segment = 100;
  double det = 1.0 / segment;
  // 判断可见性，等价于判断离散点是否在障碍物内
  for (size_t i=1; i<segment; ++i) {
    double lamb = det * i;
    Eigen::Vector2d tmp = lamb * configA + (1 - lamb) * configB;
    if (check_config_collision(tmp)) return false;
  }
  return true;
}

bool check_visibility_deformation(const std::list<Eigen::Vector2d>& pathA, const std::list<Eigen::Vector2d>& pathB) {
  return true;
}

