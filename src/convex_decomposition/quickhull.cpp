#include "convex_decomposition/quickhull.h"
#include <iostream>
#include <cfloat>

double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint) {
  return norm.dot(target-viaPoint);
}

void qkhull::quickhull(const std::vector<Eigen::Vector3d>& set) {
  int num_vertex = set.size();

  // 复制所有点到初始点集
  VertexList vertexList;
  std::shared_ptr<Vertex> tmpVtx;
  for (auto& point : set) {
    tmpVtx = std::make_shared<Vertex>();
    tmpVtx->point = point;
    vertexList.push_back(std::move(tmpVtx));
  }

  // 初始化四面体
  FaceList faceList;
  initTetrahedron(vertexList, faceList);
}

// 初始化四面体，找尽可能大的四面体
void qkhull::initTetrahedron(VertexList vertexList, FaceList facetList) {
  // 用迭代器或指针来存储链表节点的位置
  ptrVertex ptrA(*(vertexList.begin())), ptrB(*(vertexList.begin()));
  // 搜索
  for (auto& ite : vertexList) {
    // x 最小的节点
    if (ite->point[0] < ptrA->point[0]) {
      ptrA = ite;
    }
    else if (ite->point[0] == ptrA->point[0]) {
      // 保存 y 值更小的点
      ptrA = (ite->point[1] < ptrA->point[1]) ? ite : ptrA;
    }
    // x 最大的节点
    else if (ite->point[0] > ptrB->point[0]) {
      ptrB = ite;
    }
    else if (ite->point[0] == ptrB->point[0]) {
      // 保存 y 值更大的点
      ptrB = (ite->point[1] > ptrA->point[1]) ? ite : ptrA;
    }
  }
  std::cout << ptrA->point.transpose() << std::endl;
  std::cout << ptrB->point.transpose() << std::endl;

  Eigen::Vector3d dir = ptrB->point - ptrA->point;
  Eigen::Vector3d norm{dir[2],0,-dir[0]}, orth = dir.cross(norm);
  Eigen::MatrixXd projMat(3,2);
  projMat << norm, orth;

  // 找到在投影到以线段为法线的平面上最长的向量
  double tmpProj, maxProj=-1*DBL_MAX;
  ptrVertex ptrC(*(vertexList.begin()));
  for (auto& ite : vertexList) {
    tmpProj = (projMat.transpose() * ite->point).squaredNorm();
    if (tmpProj > maxProj) {
      ptrC = ite;
      maxProj = tmpProj;
    }
  }
  std::cout << ptrC->point.transpose() << std::endl;

  Eigen::Vector3d ab, ac, normABC;
  ab = ptrB->point - ptrA->point;
  ac = ptrC->point - ptrA->point;
  normABC = ab.cross(ac);

  // 找到ABC平面最远的点
  maxProj=-1*DBL_MAX;
  ptrVertex ptrD(*(vertexList.begin()));
  for (auto& ite : vertexList) {
    tmpProj = std::fabs(dist2plane(ite->point, normABC, ptrA->point));
    if (tmpProj > maxProj) {
      ptrD = ite;
      maxProj = tmpProj;
    }
  }
  std::cout << ptrD->point.transpose() << std::endl;

  std::ofstream ofile("build/data/qkhull_points", std::ios::app);
  ofile << ptrA->point.transpose() << std::endl;
  ofile << ptrB->point.transpose() << std::endl;
  ofile << ptrC->point.transpose() << std::endl;
  ofile << ptrD->point.transpose() << std::endl;
}

