#include "convex_decomposition/quickhull.h"
#include <iostream>
#include <cfloat>

void qkhull::Face::initFace(ptrVertex vtxA, ptrVertex vtxB, ptrVertex vtxC) {
  vertex[0] = vtxA;
  vertex[1] = vtxB;
  vertex[2] = vtxC;
}
void qkhull::Face::initFace(ptrVertex vtxA, ptrVertex vtxB, ptrVertex vtxC, ptrVertex vtxD) {
  vertex[0] = vtxA;
  vertex[1] = vtxB;
  vertex[2] = vtxC;
  Eigen::Vector3d ab, ac, ad;
  ab = vertex[1]->point - vertex[0]->point;
  ac = vertex[2]->point - vertex[0]->point;
  ad = vtxD->point - vertex[0]->point;
  norm = ab.cross(ac);
  if (norm.norm() == 0) {
    printf("Warning: The tetrahedron is degenerated.\n");
  }
  // d点在外法线的反方向
  norm /= norm.dot(ad)<0 ? norm.norm() : -norm.norm();
}

void qkhull::Face::setNeighbors(ptrFace faceA, ptrFace faceB, ptrFace faceC) {
  neighbor[0] = faceA;
  neighbor[1] = faceB;
  neighbor[2] = faceC;
}

void qkhull::Face::setNorm(Eigen::Vector3d vector) {
  if (vector.norm() == 0) {
    printf("Error: 0 is set as norm of a Face.\n");
  }
  norm = vector;
}

bool qkhull::Face::isAbove(ptrVertex pTarget) {
  if (norm.norm() == 0) {
    printf("Error: norm of the Face is not specified.\n");
  }
  return (norm.dot(pTarget->point - vertex[0]->point) >= 0);
}

qkhull::VertexIterator qkhull::Face::furthestVertex() {
  double maxProj = -1, tmpProj;
  VertexIterator furthestV;
  for (VertexIterator iteV=pOuterSet->vertexList.begin(); iteV!=pOuterSet->vertexList.end(); ++iteV) {
    tmpProj = dist2plane((*iteV)->point, norm, (*pOuterSet->vertexList.begin())->point);
    if (tmpProj > maxProj) {
      maxProj = tmpProj;
      furthestV = iteV;
    }
  }
  return furthestV;
}

void qkhull::Face::visit() {
  faceFlag = FaceStatus::visited;
}
bool qkhull::Face::isVisited() {
  if (faceFlag != FaceStatus::notVisit) {
    return true;
  }
  return false;
}
void qkhull::Face::resetVisit() {
  faceFlag = FaceStatus::notVisit;
}

void qkhull::Face::setBorder() {
  faceFlag = FaceStatus::borderland;
}

bool qkhull::Face::onBorder() {
  if (faceFlag == FaceStatus::borderland) {
    return true;
  }
  return false;
}

void qkhull::Edge::init(ptrFace pFaceA, ptrFace pFaceB) {
  int idx = 0;
  neighbor[0] = pFaceA;
  neighbor[1] = pFaceB;
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      if ((pFaceA->vertex[i]->point - pFaceB->vertex[j]->point).squaredNorm() == 0) {
        vertex[idx] = pFaceA->vertex[i];
        idx++;
        break;
      }
    }
  }
  if (idx == 3) {
    printf("Warning: neighbors of a Edge is the same.\n");
  }
}

void qkhull::Edge::setEndpoints(ptrVertex pVtxA, ptrVertex pVtxB) {
  vertex[0] = pVtxA;
  vertex[1] = pVtxB;
}
void qkhull::Edge::setNeighbors(ptrFace unvisibleFace, ptrFace visibleFce) {
  neighbor[0] = unvisibleFace;
  neighbor[1] = visibleFce;
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
  FaceList cvxHull;
  std::cout << "===> initTetrahedron:" << std::endl;
  initTetrahedron(vertexList, cvxHull);

  std::cout << "\nnorm of plane:" << std::endl;
  for (FaceIterator ite = cvxHull.begin(); ite != cvxHull.end(); ++ite) {
    std::cout << "norm = " << (*ite)->norm.transpose() << std::endl;
  }

  // 分配外部点集
  std::cout << "\n===> allocOuterSet:" << std::endl;
  allocOuterSet(vertexList, cvxHull);

  int i=-1, numOutPoint=0;
  for (FaceIterator ite=cvxHull.begin(); ite!=cvxHull.end(); ++ite) {
    int tmpOuterPoint = (*ite)->pOuterSet->vertexList.size();
    numOutPoint += tmpOuterPoint;
    i++;
    std::cout << "Face: " << i << " Size: ";
    std::cout << tmpOuterPoint << std::endl;
  }
  std::cout << "Num of outer point: " << numOutPoint << std::endl;

  // 更新待定面集
  std::cout << "\n===> Setting Pending Face:" << std::endl;
  FaceList pendFace;
  for (FaceIterator iteF=cvxHull.begin(); iteF!=cvxHull.end(); ++iteF) {
    // 将外部点集非空的面加入待定面集
    if ((*iteF)->pOuterSet->vertexList.size()) {
      ptrFace tmpF = *iteF;
      pendFace.emplace_back(tmpF);
    }
  }
  std::cout << "pendFace.size() = " << pendFace.size() << std::endl;

  // while (!pendFace.empty()) {
  while (pendFace.empty()) continue;
    BorderEdgeMap borderEdgeM;
    FaceList listVisibleF;

    FaceIterator pendF = pendFace.begin();
    // 找到待定面上方的最远点 p
    VertexIterator iteFurV = (*pendF)->furthestVertex();
    ptrVertex pFurVtx = *iteFurV;
    std::cout << "furthestV = " << pFurVtx->point.transpose() << std::endl;
    // 从外部点集中删除最远点
    (*pendF)->pOuterSet->vertexList.erase(iteFurV);
    std::cout << "furthestV = " << pFurVtx->point.transpose() << std::endl;

    // 找最远点的可见面，以及临界边
    std::cout << "\n===> findVisibleFace:" << std::endl;
    findVisibleFace(pFurVtx, *pendF, listVisibleF, borderEdgeM);

    // 合并可见面的点集
    std::cout << "\n===> gatherOuterSet:" << std::endl;
    VertexList visOuterSet;
    gatherOuterSet(listVisibleF, visOuterSet);
    std::cout << "visOuterSet.size() = " << visOuterSet.size() << std::endl;

    // 从待定面集删除可见面集中的面
    for (FaceIterator iteF=listVisibleF.begin(); iteF!=listVisibleF.end(); ++iteF) {
    }

    // 从临界边构建新的表面
    std::cout << "\n===> constractNewFace:" << std::endl;
    FaceList newFaceList;
    constractNewFace(pFurVtx, borderEdgeM, newFaceList);
    std::cout << newFaceList.size() << std::endl;
  // }

  std::ofstream ofile("build/data/qkhull_hull", std::ios::trunc);
  for (FaceIterator iteF=newFaceList.begin(); iteF!=newFaceList.end(); ++iteF) {
    for (int i=0; i<3; ++i) {
      ofile << (*iteF)->vertex[i]->point.transpose() << std::endl;
    }
  }
} // qkhull::quickhull()

// 初始化四面体，找尽可能大的四面体
void qkhull::initTetrahedron(VertexList& vertexList, FaceList& tetrahedron) {
  // 用迭代器或指针来存储链表节点的位置
  std::vector<VertexIterator> iteVtx(4, vertexList.begin());
  // 搜索x方向上最远的两点
  for (VertexIterator ite=vertexList.begin(); ite != vertexList.end(); ++ite) {
    // x 最小的节点
    if ((*ite)->point[0] < (*iteVtx[0])->point[0]) {
      iteVtx[0] = ite;
    }
    else if ((*ite)->point[0] == (*iteVtx[0])->point[0]) {
      // 保存 y 值更小的点
      iteVtx[0] = ((*ite)->point[1] < (*iteVtx[0])->point[1]) ? ite : iteVtx[0];
    }
    // x 最大的节点
    else if ((*ite)->point[0] > (*iteVtx[1])->point[0]) {
      iteVtx[1] = ite;
    }
    else if ((*ite)->point[0] == (*iteVtx[1])->point[0]) {
      // 保存 y 值更大的点
      iteVtx[1] = ((*ite)->point[1] > (*iteVtx[0])->point[1]) ? ite : iteVtx[0];
    }
  }
  // std::cout << (*iteVtx[0])->point.transpose() << std::endl;
  // std::cout << (*iteVtx[1])->point.transpose() << std::endl;

  Eigen::Vector3d dir = (*iteVtx[1])->point - (*iteVtx[0])->point;
  Eigen::Vector3d tmpPoint{dir[2],dir[0],dir[1]};
  Eigen::Vector3d norm = dir.cross(tmpPoint), orth = dir.cross(norm);
  Eigen::MatrixXd projMat(3,2);
  projMat << norm, orth;

  // 找到在投影到以线段为法线的平面上最长的向量
  double tmpProj, maxProj=-1*DBL_MAX;
  for (VertexIterator ite=vertexList.begin(); ite != vertexList.end(); ++ite) {
    tmpProj = (projMat.transpose() * (*ite)->point).squaredNorm();
    if (tmpProj > maxProj) {
      iteVtx[2] = ite;
      maxProj = tmpProj;
    }
  }
  // std::cout << (*iteVtx[2])->point.transpose() << std::endl;

  Eigen::Vector3d ab, ac, normABC;
  ab = (*iteVtx[1])->point - (*iteVtx[0])->point;
  ac = (*iteVtx[2])->point - (*iteVtx[0])->point;
  normABC = ab.cross(ac);

  // 找到ABC平面最远的点
  maxProj=-1*DBL_MAX;
  for (VertexIterator ite=vertexList.begin(); ite != vertexList.end(); ++ite) {
    tmpProj = std::fabs(dist2plane((*ite)->point, normABC, (*iteVtx[0])->point));
    if (tmpProj > maxProj) {
      iteVtx[3] = ite;
      maxProj = tmpProj;
    }
  }
  // std::cout << (*iteVtx[3])->point.transpose() << std::endl;

  std::ofstream ofile("build/data/qkhull_points", std::ios::app);
  ofile << (*iteVtx[0])->point.transpose() << std::endl;
  ofile << (*iteVtx[1])->point.transpose() << std::endl;
  ofile << (*iteVtx[2])->point.transpose() << std::endl;
  ofile << (*iteVtx[3])->point.transpose() << std::endl;

  // 初始化四面体的四个表面
  std::vector<ptrFace> pFace{std::make_shared<Face>(), std::make_shared<Face>(),std::make_shared<Face>(), std::make_shared<Face>()};
  // 设定表面顶点、外法线
  pFace[0]->initFace(*iteVtx[0], *iteVtx[2], *iteVtx[1], *iteVtx[3]);
  pFace[1]->initFace(*iteVtx[0], *iteVtx[1], *iteVtx[3], *iteVtx[2]);
  pFace[2]->initFace(*iteVtx[0], *iteVtx[3], *iteVtx[2], *iteVtx[1]);
  pFace[3]->initFace(*iteVtx[1], *iteVtx[2], *iteVtx[3], *iteVtx[0]);
  // 设置相临面，使相邻面的相同边方向相反
  pFace[0]->setNeighbors(pFace[2], pFace[3], pFace[1]);
  pFace[1]->setNeighbors(pFace[0], pFace[3], pFace[2]);
  pFace[2]->setNeighbors(pFace[1], pFace[3], pFace[0]);
  pFace[3]->setNeighbors(pFace[0], pFace[2], pFace[1]);

  for (int i=0; i<pFace.size(); ++i) {
    // 用四面体初始化凸包
    tetrahedron.push_back(pFace[i]);
    // 从未遍历顶点中删除四面体顶点
    vertexList.erase(iteVtx[i]);
  }
}

void qkhull::allocOuterSet(VertexList& vertexList, FaceList& faceList) {
  // 将每个顶点分配到一个表面的外部点集
  for (VertexIterator iteVtx = vertexList.begin(); iteVtx != vertexList.end(); ++iteVtx) {
    // std::cout << "Point:" << (*iteVtx)->point.transpose() << std::endl;
    int i=-1;
    for (FaceIterator iteFace = faceList.begin(); iteFace != faceList.end(); ++iteFace) {
      i++;
      if ((*iteFace)->isAbove(*iteVtx)) {
        // std::cout << "belongs to face: " << i << std::endl;
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

void qkhull::findVisibleFace(ptrVertex ptrFurV, ptrFace& ptrPendF, FaceList& listVisibleF, BorderEdgeMap& edgeM) {
  // 访问标志位置位
  ptrPendF->visit();
  // 当前面加入可见面集
  listVisibleF.push_back(ptrPendF);
  int num = 0;

  std::cout << "furthestVertex = " << ptrFurV->point.transpose() << std::endl;

  // 宽度优先搜索：从可见面集中找到临界边
  for (FaceIterator iteF=listVisibleF.begin(); iteF!=listVisibleF.end(); ++iteF) {
  // FaceIterator iteF = listVisibleF.begin();
    // std::cout << "\nVertex of cur face:" << std::endl;
    // for (int i=0; i<3; ++i) {
    //   std::cout << (*iteF)->vertex[i]->point.transpose() << std::endl;
    // }

    // 查看邻面是否可见，当前面顶点(i,i+1)对应的邻面
    for (int i=0; i<3; ++i) {
      // 创建临界边
      ptrEdge ptrE = std::make_shared<Edge>();
      ptrFace neighbor = (*iteF)->neighbor[i];

      std::cout << "\nneighbor face:" << std::endl;
      for (int j=0; j<3; ++j) {
        std::cout<< neighbor->vertex[j]->point.transpose()<<std::endl;
      }

      // 邻面未被访问过
      if (!neighbor->isVisited()) {
        neighbor->visit();
        // 该邻面可见，将其加入可见面集链表
        if (neighbor->isAbove(ptrFurV)) {
          std::cout << "Face visitible" << std::endl;
          listVisibleF.push_back(neighbor);
        }
        // 该邻面不可见，当前面和该邻面均为临界面
        else {
          num++;
          std::cout << "num of unvisibleFace = " << num  << std::endl;
          // 将该邻面设为临界面
          neighbor->setBorder();
          // ptrE->init(*iteF, neighbor);
          ptrE->setEndpoints((*iteF)->vertex[i], (*iteF)->vertex[(i+1)%3]);
          ptrE->setNeighbors(neighbor, *iteF);
          // 要确保当前map中顶点的唯一性，此处选择有向线段末端点
          edgeM.insert(std::make_pair((*iteF)->vertex[i], ptrE));
          std::cout << "edgeM.size() = " << edgeM.size() << std::endl;
        }
      } // if (!neighbor->isVisited())
      else if (neighbor->onBorder()) {
          ptrE = std::make_shared<Edge>();
          // ptrE->init(*iteF, neighbor);
          ptrE->setEndpoints((*iteF)->vertex[i], (*iteF)->vertex[(i+1)%3]);
          ptrE->setNeighbors(neighbor, *iteF);
          // 要确保当前map中顶点的唯一性
          edgeM.insert(std::make_pair((*iteF)->vertex[i], ptrE));
          std::cout << "edgeM.size() = " << edgeM.size() << std::endl;
      } // else
    }
    std::cout << "VisibleF.size() = " << listVisibleF.size() << std::endl;
  }
}

// 汇集可见面的点集
void qkhull::gatherOuterSet(FaceList& visFaceList, VertexList& visOuterSet) {
  FaceIterator iteTmpF;
  for (FaceIterator iteF=visFaceList.begin(); iteF!=visFaceList.end();) {
    // 记录当前迭代器位置
    iteTmpF = iteF;
    iteTmpF++;

    ptrOuterSet pOuterSet = (*iteF)->pOuterSet;
    if (pOuterSet) {
      visOuterSet.splice(visOuterSet.end(), pOuterSet->vertexList, pOuterSet->vertexList.begin(), pOuterSet->vertexList.end());
    }
    iteF = iteTmpF;
  }
}

void qkhull::constractNewFace(ptrVertex pVertex, BorderEdgeMap& borderEdgeM, FaceList& newFaceL) {
  std::cout << "borderEdgeM.size() = " << borderEdgeM.size() << std::endl;
  for (EdgeIterator iteE=borderEdgeM.begin(); iteE!=borderEdgeM.end(); ++iteE) {
    std::cout << (*iteE).second->vertex[1]->point.transpose() << std::endl;
  }
  EdgeIterator curE = borderEdgeM.begin(), nextE;
  ptrEdge pEdge;
  Eigen::Vector3d norm, ab, ac, ad;

  while (!borderEdgeM.empty()) {
    pEdge = curE->second;

    // 构建新表面
    ptrFace pFace = std::make_shared<Face>();
    pFace->initFace(pVertex, pEdge->vertex[0], pEdge->vertex[1]);
    // 更新新表面的相邻面
    pFace->neighbor[1] = pEdge->neighbor[0];
    // 将新表面加入链表
    newFaceL.push_back(pFace);

    // 更新临界边上不可见面的邻面
    for (int i=0; i<3; ++i) {
      if (pEdge->neighbor[0]->neighbor[i] == pEdge->neighbor[1]) {
        pEdge->neighbor[0]->neighbor[i] = pFace;
        break;
      }
    }
    // 重置访问标志
    pEdge->neighbor[0]->resetVisit();

    // 按顺序找下一条临界边
    borderEdgeM.erase(curE);
    curE = borderEdgeM.find(pEdge->vertex[1]);
  }
  std::cout << "after edgeM.size() = " << borderEdgeM.size() << std::endl;
  std::cout << "newFaceL.size() = " << newFaceL.size() << std::endl;

  // 更新新表面的另外两个相邻面，环形循环
  FaceIterator iteCurF=newFaceL.begin(), iteLastF = newFaceL.end();
  iteLastF--;
  for (; iteCurF != newFaceL.end(); iteLastF = iteCurF, iteCurF++) {
    // 设定相邻面
    (*iteCurF)->neighbor[0] = *iteLastF;
    (*iteLastF)->neighbor[2] = *iteCurF;

    // 设定新表面的法线，上条边的起点在当前面的下方
    ab = (*iteCurF)->vertex[1]->point - (*iteCurF)->vertex[0]->point;
    ac = (*iteCurF)->vertex[2]->point - (*iteCurF)->vertex[0]->point;
    ad = (*iteLastF)->vertex[1]->point - (*iteCurF)->vertex[0]->point;
    norm = ab.cross(ac);
    norm /= norm.dot(ad)<0 ? norm.norm() : -norm.norm();
    (*iteCurF)->setNorm(norm);
  }
}
