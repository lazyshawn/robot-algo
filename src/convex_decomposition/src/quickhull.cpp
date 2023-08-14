// convex_decomposition
#include "convex_decomposition/quickhull.h"

const double posInf = std::numeric_limits<double>::infinity();
const double negInf = -std::numeric_limits<double>::infinity();

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

bool qkhull::Face::visited() {
  if (faceFlag != FaceStatus::notVisited) {
    return true;
  }
  return false;
}
void qkhull::Face::setVisible() {
  faceFlag = FaceStatus::visible;
}
bool qkhull::Face::isVisible() {
  if (faceFlag != FaceStatus::visible) {
    return true;
  }
  return false;
}
void qkhull::Face::resetVisit() {
  faceFlag = FaceStatus::notVisited;
}

void qkhull::Face::setBorder() {
  faceFlag = FaceStatus::border;
}

bool qkhull::Face::onBorder() {
  if (faceFlag == FaceStatus::border) {
    return true;
  }
  return false;
}

void qkhull::Edge::setEndpoints(ptrVertex pVtxA, ptrVertex pVtxB) {
  vertex[0] = pVtxA;
  vertex[1] = pVtxB;
}
void qkhull::Edge::setNeighbors(ptrFace unvisibleFace, ptrFace visibleFce) {
  neighbor[0] = unvisibleFace;
  neighbor[1] = visibleFce;
}

void qkhull::quickhull(const std::vector<Eigen::Vector3d>& set, FaceList& cvxHull) {
  // 复制所有点到初始点集
  VertexList vertexList;
  std::shared_ptr<Vertex> tmpVtx;
  for (auto& point : set) {
    tmpVtx = std::make_shared<Vertex>();
    tmpVtx->point = point;
    vertexList.push_back(std::move(tmpVtx));
  }

  // 初始化四面体
  FaceList tetrahedron;
  initTetrahedron(vertexList, tetrahedron);

  // 分配外部点集
  allocOuterSet(vertexList, tetrahedron);

  // 更新待定面集
  FaceList pendFace;
  updatePendFace(tetrahedron, pendFace, cvxHull);

  while (!pendFace.empty()) {
    BorderEdgeMap borderEdgeM;
    FaceList listVisibleF;

    FaceIterator pendF = pendFace.begin();
    // 找到待定面上方的最远点 p
    VertexIterator iteFurV = (*pendF)->furthestVertex();
    ptrVertex pFurVtx = *iteFurV;
    // 从外部点集中删除最远点
    (*pendF)->pOuterSet->vertexList.erase(iteFurV);

    // 找最远点的可见面，以及临界边
    findVisibleFace(pFurVtx, *pendF, listVisibleF, borderEdgeM);

    // 合并可见面的点集
    VertexList visOuterSet;
    gatherOuterSet(listVisibleF, visOuterSet);

    // 从临界边构建新的表面
    FaceList newFaceList;
    constractNewFace(pFurVtx, borderEdgeM, newFaceList);

    // 将可见面的外部点集分配到新表面
    allocOuterSet(visOuterSet, newFaceList);

    // 从待定面集和确定集中删除可见面集中的面
    removeVisibleFace(pendFace, cvxHull);

    // 更新待定面集
    updatePendFace(newFaceList, pendFace, cvxHull);
  } // while (!pendFace.empty())

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

  Eigen::Vector3d dir = (*iteVtx[1])->point - (*iteVtx[0])->point;
  // 与 dir 不共面的任意向量，包含 dir 的一组基底的三个基向量之和
  Eigen::Vector3d tmpPoint{dir[0]-dir[1]-dir[2], dir[0]+dir[1], dir[0]+dir[2]};
  Eigen::Vector3d norm = dir.cross(tmpPoint), orth = dir.cross(norm);
  Eigen::MatrixXd projMat(3,2);
  projMat << norm, orth;

  // 找到在投影到以线段为法线的平面上最长的向量
  double tmpProj, maxProj=negInf;
  for (VertexIterator ite=vertexList.begin(); ite != vertexList.end(); ++ite) {
    tmpProj = (projMat.transpose() * (*ite)->point).squaredNorm();
    if (tmpProj > maxProj) {
      iteVtx[2] = ite;
      maxProj = tmpProj;
    }
  }

  Eigen::Vector3d ab, ac, normABC;
  ab = (*iteVtx[1])->point - (*iteVtx[0])->point;
  ac = (*iteVtx[2])->point - (*iteVtx[0])->point;
  normABC = ab.cross(ac);

  // 找到ABC平面最远的点
  maxProj=negInf;
  for (VertexIterator ite=vertexList.begin(); ite != vertexList.end(); ++ite) {
    tmpProj = std::fabs(dist2plane((*ite)->point, normABC, (*iteVtx[0])->point));
    if (tmpProj > maxProj) {
      iteVtx[3] = ite;
      maxProj = tmpProj;
    }
  }

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

  for (int i = 0; i < static_cast<int>(pFace.size()); ++i) {
    // 用四面体初始化凸包
    tetrahedron.push_back(pFace[i]);
    // 从未遍历顶点中删除四面体顶点
    vertexList.erase(iteVtx[i]);
  }
}

void qkhull::findVisibleFace(ptrVertex ptrFurV, ptrFace& ptrPendF, FaceList& listVisibleF, BorderEdgeMap& edgeM) {
  // 访问标志位置位
  ptrPendF->setVisible();
  // 当前面加入可见面集
  listVisibleF.push_back(ptrPendF);

  // 宽度优先搜索：从可见面集中找到临界边
  for (FaceIterator iteF=listVisibleF.begin(); iteF!=listVisibleF.end(); ++iteF) {
    // 查看邻面是否可见，当前面顶点(i,i+1)对应的邻面
    for (int i=0; i<3; ++i) {
      // 创建临界边
      ptrEdge ptrE = std::make_shared<Edge>();
      ptrFace neighbor = (*iteF)->neighbor[i];

      // 邻面未被访问过
      if (!neighbor->visited()) {
        // 该邻面可见，将其加入可见面集链表
        if (neighbor->isAbove(ptrFurV)) {
          neighbor->setVisible();
          listVisibleF.push_back(neighbor);
        }
        // 该邻面不可见，当前面和该邻面均为临界面
        else {
          // 将该邻面设为临界面
          neighbor->setBorder();
          ptrE->setEndpoints((*iteF)->vertex[i], (*iteF)->vertex[(i+1)%3]);
          ptrE->setNeighbors(neighbor, *iteF);
          // 要确保当前map中顶点的唯一性，此处选择有向线段末端点
          edgeM.insert(std::make_pair((*iteF)->vertex[i], ptrE));
        }
      } // if (!neighbor->isVisited())
      else if (neighbor->onBorder()) {
          ptrE = std::make_shared<Edge>();
          ptrE->setEndpoints((*iteF)->vertex[i], (*iteF)->vertex[(i+1)%3]);
          ptrE->setNeighbors(neighbor, *iteF);
          // 要确保当前map中顶点的唯一性
          edgeM.insert(std::make_pair((*iteF)->vertex[i], ptrE));
      } // else
    }
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

void qkhull::removeVisibleFace(FaceList& listPendFace, FaceList& listFinishFace) {
  FaceIterator iteTmpF;
  for (FaceIterator iteF=listPendFace.begin(); iteF!=listPendFace.end();) {
    iteTmpF = iteF;
    iteTmpF++;
    if ((*iteF)->faceFlag==FaceStatus::visible) {
      listPendFace.erase(iteF);
    }
    iteF = iteTmpF;
  }
  for (FaceIterator iteF=listFinishFace.begin(); iteF!=listFinishFace.end();) {
    iteTmpF = iteF;
    iteTmpF++;
    if ((*iteF)->faceFlag==FaceStatus::visible) {
      listFinishFace.erase(iteF);
    }
    iteF = iteTmpF;
  }
}

void qkhull::updatePendFace(FaceList& listNewF, FaceList& listPendF, FaceList& listFinishF) {
  for (FaceIterator iteF=listNewF.begin(); iteF!=listNewF.end(); ++iteF) {
    if ((*iteF)->pOuterSet->vertexList.size()) {
      // 将外部点集非空的面加入待定面集
      ptrFace tmpF = *iteF;
      listPendF.push_back(tmpF);
    } else {
      // 记录外部点集为空的表面
      ptrFace tmpF = *iteF;
      listFinishF.push_back(tmpF);
    }
  }
  listNewF.clear();
}

