#include "collision_detection/gjk.h"

// 调试用
#include <iostream>

// 找凸集中在给定方向上的最远点
Eigen::Vector3d farthest_point_on_direction(std::vector<Eigen::Vector3d> convexSet, Eigen::Vector3d dir) {
  if (convexSet.empty()) printf("the given convex set is empty!\n");

  int num_vertex = convexSet.size();
  double maxProj = -1*DBL_MAX, tmpProj = 0.0;
  Eigen::Vector3d farthestPoint{0,0,0};

  // 遍历每个顶点，对比顶点在给定方向上的投影，找到最远的点
  for (auto& vertex : convexSet) {
    tmpProj = vertex.transpose() * dir;
    if (tmpProj > maxProj) {
      maxProj = tmpProj;
      farthestPoint = vertex;
    }
  }
  return farthestPoint;
}

// gjk_support 函数，找到两几何体在给定方向上最大的闵可夫斯基差
Eigen::Vector3d gjk_support(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB, Eigen::Vector3d dir) {
  Eigen::Vector3d vertexA, vertexB;
  vertexA = farthest_point_on_direction(setA, dir);
  vertexB = farthest_point_on_direction(setB, -dir);
  return vertexA-vertexB;
}

// 更新单纯性和搜索方向
Eigen::Vector3d gjk_collision_update(std::vector<Eigen::Vector3d>& simplexList, Eigen::Vector3d support) {
  // 原单纯形中的顶点数量
  int num_vertex = simplexList.size();
  // 结束时的搜索方向
  Eigen::Vector3d dir{0,0,0};
  // 单纯形的特定边，s: Support, abc: 单纯形中第一至三个顶点
  Eigen::Vector3d sa, sb, sc, norm;
  double tolerance = 1e-9;

  if (num_vertex == 1) {
    // 将新的 Support 点加入单纯形
    simplexList.emplace_back(support);
    // 两点连线的指向原点法向量
    sa = simplexList[0] - support;
    dir = -sa.cross(sa.cross(support));
    // 保证法线方向指向原点
    return (dir.dot(support) > 0) ? -dir : dir;
  }
  else if (num_vertex == 2) {
    // 线段、原点、Support点在同一平面
    if (fabs(simplexList[0].cross(simplexList[1]).dot(support)) < tolerance) {
      // 原点将在 Support 与单纯形中一点的连线外侧
      for (int i=0; i<num_vertex; ++i) {
        sa = simplexList[i] - support;
        sb = simplexList[(i+1)%num_vertex] - support;
        norm = sa.cross(sa.cross(sb));
        // 在三角形sb边的外侧
        if (norm.dot(-support) > 0) {
          // 替换远离原点的顶点
          simplexList[(i+1)%num_vertex] = support;
          dir = -sa.cross(sa.cross(support));
          return (dir.dot(support) > 0) ? -dir : dir;
        }
      }
      // 原点在每个侧面的内侧，即三角形包含原点
      return dir;
    } else {
      // 不在同一平面，Support 点加入单纯形
      simplexList.emplace_back(support);
      // 平面法向量
      dir = (simplexList[0]-support).cross(simplexList[1]-support);
      return (dir.dot(support) > 0) ? -dir : dir;
    }
  }
  // 单纯形中已有三个顶点
  for (int i = 0; i < 3; ++i) {
    sa = simplexList[i] - support;
    sb = simplexList[(i + 1) % num_vertex] - support;
    sc = simplexList[(i + 2) % num_vertex] - support;
    norm = sa.cross(sb);
    if (norm.dot(sc) > 0)
      norm *= -1;
    // 原点在 sab 的外侧，将 c 点替换为 Support
    if (norm.dot(sc) * norm.dot(-support) < 0) {
      simplexList[(i + 2) % num_vertex] = support;
      // 返回此时单纯形平面的法线
      return norm;
    }
  }
  // 包含原点
  return dir;
}

bool gjk_collision_detection(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB) {
  int maxIte = 100;
  // 第一次搜索方向
  Eigen::Vector3d dir = (setA[0] - setB[0]);
  // 第一个 Support 点
  Eigen::Vector3d support = gjk_support(setA, setB, dir);
  // 单纯形
  std::vector<Eigen::Vector3d> simplexList; 
  simplexList.emplace_back(support);
  // 第二次搜索
  dir *= -1;

  // 开始循环
  for (int i=0; i<maxIte; ++i) {
    // 新的 Support 点
    support = gjk_support(setA, setB, dir);

    // 不能再找到跨越原点的点
    if (support.dot(dir) < 0) {
      return false;
    }
    // 更新单纯形和搜索方向
    dir = gjk_collision_update(simplexList, support);
    if (dir.norm() == 0) {
      return true;
    }

    if (i==maxIte) {printf("gjk_collision_detection: excessive loop!\n");}
  }
  return false;
}

// 计算未碰撞的凸集上的最近距离
double gjk_closest_distance(std::vector<Eigen::Vector3d> setA, std::vector<Eigen::Vector3d> setB) {
  // 构造初始单纯形
  std::vector<Eigen::Vector3d> simplexList;
  // 第一次搜索方向和对应的 Suppor 点
  Eigen::Vector3d dir = setA[0] - setB[0], closestPoint;
  // 第一个 Support 点
  Eigen::Vector3d support = gjk_support(setA, setB, dir);
  simplexList.emplace_back(support);
  // 第二次搜索方向，开始进入循环
  dir = -support;

  // 最大迭代次数
  int maxIte = 100;
  // 允许误差，解决由于精度原因两内积之差足够小但不为零的问题
  double tolerance = 1e-9;
  for (int i=0; i<maxIte; ++i) {
    Eigen::Vector3d newSpt = gjk_support(setA, setB, dir);
    // 新 Support 点和单纯形在同一个平面，不可以组成离原点更近的面
    if (dir.dot(newSpt) - dir.dot(support) < tolerance && simplexList.size() > 1) {
      break;
    }
    // 更新 Support 点
    support = newSpt;
    // 更新单纯形和搜索方向(返回值为最近点，取反后即为搜索方向)
    closestPoint = gjk_closest_distance_update(simplexList, support);
    dir = -1 * closestPoint/closestPoint.norm();
  }
  return closestPoint.norm();
}

Eigen::Vector3d closest_point_on_line(std::vector<Eigen::Vector3d> simplexList) {
  int n = simplexList.size();
  if (n - 2 != 0) {
    printf("Error: wrong number vertex on a line!\n");
    return {0,0,0};
  }
  Eigen::Vector3d oa = simplexList[0], ob = simplexList[1];
  Eigen::Vector3d ab = ob - oa;

  if (oa.dot(ab) >= 0) {
    // 在 A 点外侧
    return simplexList[0];
  } else if (ob.dot(ab) <= 0) {
    // 在 B 点外侧
    return simplexList[1];
  }
  return oa + (-oa).dot(ab)*ab/ab.squaredNorm();
}

Eigen::Vector3d closest_point_on_face(std::vector<Eigen::Vector3d> simplexList) {
  int num_vertex = simplexList.size();
  if (num_vertex - 3 != 0) {
    printf("Error: wrong number vertex on a face!\n");
    return {0,0,0};
  }
  Eigen::Vector3d ab, ac, ao, bo, co, norm;
  for (int i=0; i<num_vertex; ++i) {
    ao = -simplexList[i];
    bo = -simplexList[(i+1)%num_vertex];
    co = -simplexList[(i+2)%num_vertex];
    ab = ao - bo;
    ac = ao - co;
    // ab 边外法线
    norm = ab.cross(ab.cross(ac));

    // 原点在 ab 边内侧，注意在之后考虑等于零的情况
    if (norm.dot(ao) < 0) continue;

    if (ao.dot(ab) < 0) {
      // 原点在 a 外侧，最近点在 ac 上
      return closest_point_on_line({-ao, -co});
    } else if (ao.dot(ab) > ab.squaredNorm()) {
      // 原点在 b 外侧，最近点在 bc 上
      return closest_point_on_line({-bo, -co});
    }
    // 原点在 ab 内，最近点在 ab 上
    return closest_point_on_line({-ao, -bo});
  }

  // 最近点在三角形内
  norm = ab.cross(ac);
  // 保证法线指向原点
  if (norm.dot(ao) < 0) norm *= -1;
  // 最近点即为 oa 在法线上的投影
  return -ao.dot(norm)*norm/norm.squaredNorm();
}

// 返回最近点的坐标
Eigen::Vector3d gjk_closest_distance_update(std::vector<Eigen::Vector3d>& simplexList, Eigen::Vector3d support){
  Eigen::Vector3d norm, sa, sb;
  // 与 Support 组成的到原点最近的边的顶点序号
  int closestSideIdx = -1;
  int num_vertex = simplexList.size();
  double tolerance = 1e-9;

  // 如果单纯形中只有一个点，则将 Support 点加入其中
  if (num_vertex == 1) {
    simplexList.emplace_back(support);
    // 搜索方向为线段上到原点的最近点
    return closest_point_on_line(simplexList);
  }

  // 如果单纯形中有两个点，判断当前算法的维度
  if (num_vertex == 2) {
    sa = simplexList[0] - support;
    sb = simplexList[1] - support;
    norm = sa.cross(sb);
    // 在同一个维度
    if (fabs(support.dot(norm)) < tolerance) {
      if (support.dot(sa) > 0) {
        // 原点在 sb 方向上，将 a 替换为 Support
        simplexList[0] = support;
      } else if (support.dot(sb) > 0) {
        // 原点在 sa 方向上，将 b 替换为 Support
        simplexList[1] = support;
      }
      return closest_point_on_line(simplexList);
    } else {
      simplexList.emplace_back(support);
      // 三角形平面到原点的最近点
      return closest_point_on_face(simplexList);
    }
  }

  // 此时 Support 可以组成一个面到原点的距离更近的面，我们需要找到这个面
  for (int i=0; i<num_vertex; ++i) {
    // 面的法向量
    norm = (simplexList[i] - support).cross(simplexList[(i+1)%num_vertex] - support);
    // 确保法向量指向外侧
    if (norm.dot(simplexList[(i + 2) % num_vertex] - support) > 0) norm *= -1;
    // 如果原点在面内侧，则第四个点必定在最近的面上
    if (norm.dot(-support) < 0) {
      closestSideIdx = (i + 2) % num_vertex;
      break;
    }
  }
  // 原点在三个侧面的外侧，最近点即为 Support 点
  if (closestSideIdx <  0) return support;

  // 判断原点的投影是否在侧面三角形内
  sa = simplexList[closestSideIdx] - support;
  sb = simplexList[(closestSideIdx+1)%num_vertex] - support;
  // 面 sab 在 sa 边上的外法线方向
  norm = sa.cross(sa.cross(sb));
  // 原点在 sab 面外侧，最近的在另一个面上
  if (norm.dot(support) < 0) {
    // 将 b 点替换为 Support
    simplexList[(closestSideIdx+1)%num_vertex] = support;
  } else {
    // 将 c 点替换为 Support
    simplexList[(closestSideIdx+2)%num_vertex] = support;
  }
  return closest_point_on_face(simplexList);
}

