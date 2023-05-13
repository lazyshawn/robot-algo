#pragma once
#include <eigen3/Eigen/Dense>
#include <memory>
// 存储点集和面集，插入、删除为O(1)
#include <list>

#include "user_interface/fileio.h"

namespace qkhull {
  class Vertex;
  class Edge;
  class Face;
  class OuterSet;

  typedef std::shared_ptr<Vertex> ptrVertex;
  typedef std::list<ptrVertex> VertexList;
  typedef std::shared_ptr<Face> ptrFace;
  typedef std::list<std::shared_ptr<Face>> FaceList;

  class Vertex {
    public:
      bool onHull = false;
      Eigen::Vector3d point{0,0,0};
      Vertex() { }
      Vertex(Vertex& vtx) {
        onHull = vtx.onHull;
        point = vtx.point;
      }
  };

  class Edge {

  };

  class OuterSet {
    public:
      VertexList vertexList;
      ~OuterSet() {
        vertexList.clear();
      }
  };

  /* 
  * @brief : 快速凸包算法
  * @param : set: 点集
  * @return: 
  * @ref   : https://zhuanlan.zhihu.com/p/166105080
  */
  void quickhull(const std::vector<Eigen::Vector3d>& set);
  void initTetrahedron(VertexList vertexList, FaceList facetList);

} // namespace qkhull

/* 
* @brief : 点到平面的有向距离
* @param : target: 目标点
* @param : norm: 平面的外法向量(需先单位化)
* @param : viaPoint: 平面任意一点
* @return: 有向距离
*/
double dist2plane(Eigen::Vector3d target, Eigen::Vector3d norm, Eigen::Vector3d viaPoint);

