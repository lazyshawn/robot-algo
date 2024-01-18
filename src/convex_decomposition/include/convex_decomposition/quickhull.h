/**
* @file   quickhull.h
* @brief  Quickhull
*
* 三维点集的 quickhull 算法
*/
#pragma once
#include <eigen3/Eigen/Dense>
#include <memory>
// 存储点集和面集，插入、删除为O(1)
#include <list>
// 存储临界边
#include <map>

#include "geometry/algebra.h"
#include "geometry/geometry.h"

/**
* @brief  quickhull
*/
namespace qkhull {
  class Vertex;
  class Edge;
  class Face;
  class OuterSet;
  class Hull;

  //! 顶点链表
  typedef std::shared_ptr<Vertex> ptrVertex;
  typedef std::list<ptrVertex> VertexList;
  typedef VertexList::iterator VertexIterator;
  // 表面链表
  typedef std::shared_ptr<Face> ptrFace;
  typedef std::list<std::shared_ptr<Face>> FaceList;
  typedef FaceList::iterator FaceIterator;
  // 外部点集
  typedef std::shared_ptr<OuterSet> ptrOuterSet;

  typedef std::shared_ptr<Edge> ptrEdge;
  typedef std::map<ptrVertex, ptrEdge> BorderEdgeMap;
  typedef BorderEdgeMap::iterator EdgeIterator;

  /**
  * @brief  表面的可见性
  */
  enum class FaceStatus {
    notVisited, /**< 不可见面 */
    visible,    /**< 可见面 */
    border      /**< 边界面 */
  };

  /**
  * @brief  几何元素，点
  */
  class Vertex {
    public:
      bool onHull = false;
      Eigen::Vector3d point{0,0,0};

      Vertex() { }
      Vertex(Vertex& vtx) {
        onHull = vtx.onHull;
        point = vtx.point;
      }
      void clear();
      void erase();
  };

  /**
  * @brief  几何元素，边
  */
  class Edge {
    public:
      std::vector<ptrVertex> vertex = std::vector<ptrVertex>(2);
      std::vector<ptrFace> neighbor = std::vector<ptrFace>(2);

    public:
      void setEndpoints(ptrVertex pVtxA, ptrVertex pVtxB);
      void setNeighbors(ptrFace unvisibleFace, ptrFace visibleFce);
  };

  /**
  * @brief  几何元素，三角形面
  */
  class Face {
    public:
      // 三角形表面的三个顶点
      std::vector<ptrVertex> vertex = std::vector<ptrVertex>(3);
      // 三个相邻表面
      std::vector<ptrFace> neighbor = std::vector<ptrFace>(3);
      // 外部点集
      ptrOuterSet pOuterSet;
      // 正法线
      Eigen::Vector3d norm{0,0,0};
      // 表面访问标志
      FaceStatus faceFlag = FaceStatus::notVisited;

    public:
      Face() {
        pOuterSet = std::make_shared<OuterSet>();
      }
      void initFace(ptrVertex vtxA, ptrVertex vtxB, ptrVertex vtxC);
      void initFace(ptrVertex vtxA, ptrVertex vtxB, ptrVertex vtxC, ptrVertex vtxD);
      void setNeighbors(ptrFace faceA, ptrFace faceB, ptrFace faceC);
      void setNorm(Eigen::Vector3d norm);
      bool isAbove(ptrVertex point);
      VertexIterator furthestVertex();
      bool visited();
      void setVisible();
      bool isVisible();
      void resetVisit();
      void setBorder();
      bool onBorder();
  };

  /**
  * @brief  点集
  *
  * 用于表面上方的点
  */
  class OuterSet {
    public:
      VertexList vertexList;
    public:
      // OuterSet() {}
      ~OuterSet() {
        vertexList.clear();
      }
  };

  /**
  * @brief  凸包
  */
  class Hull {
    public:
      VertexList vertex;
      FaceList face;

    public:
      void init();
  };

  /**
  * @brief  快速凸包算法
  * @param  set 点集
  * @param  [out] cvxHull 凸包, 面集
  * @see    https://zhuanlan.zhihu.com/p/166105080
  */
  void quickhull(const std::vector<Eigen::Vector3d>& set, FaceList& cvxHull);
  /**
  * @brief  初始化四面体，找尽可能大的四面体
  * @param  [out] vertexList 四面体的顶点链表
  * @param  [out] faceList 四面体的面链表
  */
  void initTetrahedron(VertexList& vertexList, FaceList& tetrahedron);
  /**
  * @brief  为平面分配外部点集
  * @param  vertexList 待分配的点集
  * @param  face 待分配的表面
  */
  void allocOuterSet(VertexList& vertexList, FaceList& face);
  /**
  * @brief  移除可见面
  *
  * 可见面一定不是凸包所含的面，故排除
  *
  * @param  listPendFace 待定面集的链表
  * @param  listFinishFace 确定面集的链表
  */
  void removeVisibleFace(FaceList& listPendFace, FaceList& listFinishFace);
  /**
  * @brief  更新待定面集
  * @param  listNewF    新加入的面
  * @param  listPendF   待定面集
  * @param  listFinishF 确定面集
  */
  void updatePendFace(FaceList& listNewF, FaceList& listPendF, FaceList& listFinishF);
  /**
  * @brief  找最远点的可见面集
  *
  * 宽度优先搜索，判断在最远点处每个面的可见性; 
  * 可见面与不可见面的邻边为临界边
  *
  * @param  ptrFurV      最远点
  * @param  ptrPendF     当前搜索面
  * @param  listVisibleF 可见面链表
  * @param  edgeM        临界边
  */
  void findVisibleFace(ptrVertex ptrFurV, ptrFace& ptrPendF, FaceList& listVisibleF, BorderEdgeMap& edgeM);
  /**
  * @brief  合并可见面的外部点集
  * @param  visFaceList 可见面
  * @param  visOuterSet 可见面的所有外部点集
  */
  void gatherOuterSet(FaceList& visFaceList, VertexList& visOuterSet);
  /**
  * @brief  用最远点和临界边构建新的表面
  * @param  pVertex     最远点
  * @param  borderEdgeM 临界边
  * @param  newFaceL    新的表面
  */
  void constractNewFace(ptrVertex pVertex, BorderEdgeMap& borderEdgeM, FaceList& newFaceL);
} // namespace qkhull

