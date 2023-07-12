// convex_decomposition
#include "convex_decomposition/convex_decomposition.h"
#include "user_interface/user_interface.h"

using namespace geometry;

Timer timer;
std::vector<Eigen::Vector3d> pointSet;
std::ofstream pointFile("build/data/qkhull_points", std::ios::trunc);
Eigen::MatrixXd randomMat = get_random_matrix(3, 10, 0, 10);

qkhull::FaceList cvxHull;
std::vector<Vertex> vertices;
std::vector<ptrFace> faces;

void find_feasible_tetrahedron(std::list<Eigen::Vector3d>& pointVec, std::vector<Vertex>& vertices);
void init_tetrahedron(const std::vector<Vertex>& vertices, std::vector<ptrFace>& faces);

void test_quickhull();

void save_hull();

int main(int argc, char** argv) {
  std::cout << randomMat << "\n" << std::endl;
  std::list<Eigen::Vector3d> pointList;
  for (size_t i=0; i<randomMat.cols(); ++i) {
    pointSet.emplace_back(randomMat.col(i));
    pointList.emplace_back(randomMat.col(i));
  }

  find_feasible_tetrahedron(pointList, vertices);
  init_tetrahedron(vertices, faces);
  // 分配外部点集

  // test_quickhull();

  save_hull();
  return 0;
}

void find_feasible_tetrahedron(std::list<Eigen::Vector3d>& pointList, std::vector<Vertex>& vertices) {
  vertices = std::vector<Vertex>(4, pointList.front());
  typedef std::list<Eigen::Vector3d>::iterator itePointList;
  std::vector<itePointList> iteVec(4, pointList.begin());

  // 搜索x方向上最远的两点
  for (itePointList ite=pointList.begin(); ite!=pointList.end(); ++ite) {
    // x 最小的节点
    if ((*ite)[0] < vertices[0][0]) {
      vertices[0] = *ite;
      iteVec[0] = ite;
    }
    else if ((*ite)[0] == vertices[0][0]) {
      // 保存 y 值更小的点
      vertices[0] = ((*ite)[1] < vertices[0][1]) ? *ite : vertices[0];
      iteVec[0] = ite;
    }
    // x 最大的节点
    if ((*ite)[0] > vertices[1][0]) {
      vertices[1] = *ite;
      iteVec[1] = ite;
    }
    else if ((*ite)[0] == vertices[1][0]) {
      // 保存 y 值更大的点
      vertices[1] = ((*ite)[1] > vertices[1][1]) ? *ite : vertices[1];
      iteVec[1] = ite;
    }
  }

  Eigen::Vector3d dir = vertices[1] - vertices[0];
  Eigen::Vector3d norm{-dir[1]-dir[2],dir[0],dir[0]};
  Eigen::Vector3d orth = dir.cross(norm);
  Eigen::MatrixXd projMat(3,2);
  projMat << norm, orth;

  // 找到在投影到以线段为法线的平面上最长的向量
  double maxProj = -1;
  for (itePointList ite=pointList.begin(); ite!=pointList.end(); ++ite) {
    double tmpProj = (projMat.transpose() * (*ite)).squaredNorm();
    if (tmpProj > maxProj) {
      vertices[2] = *ite;
      iteVec[2] = ite;
      maxProj = tmpProj;
    }
  }

  // 根据已经确定的三个顶点构造平面
  Eigen::Vector3d ab = vertices[1] - vertices[0];
  Eigen::Vector3d ac = vertices[2] - vertices[0];
  Eigen::Vector3d normABC = ab.cross(ac);

  // 找到距离 ABC 平面最远的点
  maxProj = -1;
  for (itePointList ite=pointList.begin(); ite!=pointList.end(); ++ite) {
    double tmpProj = std::fabs(dist2plane((*ite), normABC, vertices[0]));
    if (tmpProj > maxProj) {
      vertices[3] = *ite;
      iteVec[3] = ite;
      maxProj = tmpProj;
    }
  }

  // for (size_t i=0; i<4; ++i) {
  //   std::cout << vertices[i].transpose() << std::endl;
  // }
  for (size_t i = 0; i < vertices.size(); ++i) {
    // 从未遍历顶点中删除四面体顶点
    pointList.erase(iteVec[i]);
  }
}

void init_tetrahedron(const std::vector<Vertex>& vertices, std::vector<ptrFace>& faces) {
  const int numOfVertex = 4;
  faces = std::vector<ptrFace>(numOfVertex);
  std::vector<Vertex> orderedVertex = vertices;

  for (size_t i=0; i<numOfVertex; ++i) {
    Eigen::Vector3d ab = vertices[(i+1)%numOfVertex] - vertices[i];
    Eigen::Vector3d ac = vertices[(i+2)%numOfVertex] - vertices[i];
    Eigen::Vector3d ad = vertices[(i+3)%numOfVertex] - vertices[i];

    if (double volume = ab.cross(ac).dot(ad); volume < 0) {
      for (size_t j=0; j<numOfVertex; ++j) {
        orderedVertex[j] = vertices[(i+j)%numOfVertex];
      }
      break;
    }
  }

  faces[0] = std::make_shared<Triangle>(std::vector<Vertex>{orderedVertex[0], orderedVertex[1], orderedVertex[2]});
  faces[1] = std::make_shared<Triangle>(std::vector<Vertex>{orderedVertex[1], orderedVertex[3], orderedVertex[2]});
  faces[2] = std::make_shared<Triangle>(std::vector<Vertex>{orderedVertex[2], orderedVertex[3], orderedVertex[0]});
  faces[3] = std::make_shared<Triangle>(std::vector<Vertex>{orderedVertex[3], orderedVertex[1], orderedVertex[0]});

  for (size_t i=0; i<numOfVertex; ++i) {
    faces[i]->construct_halfEdge();
    // std::cout << "\n===> Face: " << i << std::endl;
    // std::cout << "norm = " << faces[i]->normal.transpose() << std::endl;
    // for (size_t j=0; j<faces[i]->vertex.size(); ++j) {
    //   std::cout << faces[i]->vertex[j].transpose() << std::endl;
    // }
  }
}

void qkhull::allocOuterSet(VertexList& vertexList, FaceList& faceList) {
  // 将每个顶点分配到一个表面的外部点集
  for (VertexIterator iteVtx = vertexList.begin(); iteVtx != vertexList.end(); ++iteVtx) {
    for (FaceIterator iteFace = faceList.begin(); iteFace != faceList.end(); ++iteFace) {
      if ((*iteFace)->isAbove(*iteVtx)) {
        ptrVertex tmpVtx = std::make_shared<Vertex>();
        tmpVtx = (*iteVtx);
        (*iteFace)->pOuterSet->vertexList.emplace_back((tmpVtx));
        break;
      }
    }
  }
  // clear vertexList
  vertexList.clear();
}
void test_quickhull() {
  timer.start();
  qkhull::quickhull(pointSet, cvxHull);
  std::cout << "Total time: " << timer.ms_since_last() << "ms" << std::endl;
}

void save_hull() {
  for (size_t i=0; i<randomMat.cols(); ++i) {
    pointFile << pointSet[i].transpose() << std::endl;
  }

  // 记录凸包表面
  std::ofstream hullFile("build/data/qkhull_hull", std::ios::trunc);
  // for (qkhull::FaceIterator iteF=cvxHull.begin(); iteF!=cvxHull.end(); ++iteF) {
  //   for (int i=0; i<3; ++i) {
  //     hullFile << (*iteF)->vertex[i]->point.transpose() << std::endl;
  //   }
  // }
  for (auto& face : faces) {
    for (int i=0; i<3; ++i) {
      hullFile << face->vertex[i].transpose() << std::endl;
    }
  }
}

