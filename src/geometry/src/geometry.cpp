#include "geometry/geometry.h"

geometry::HalfEdge::HalfEdge (Vertex begPoint, Vertex endPoint) {
  point = std::vector<Vertex>{begPoint, endPoint};
  face = std::vector<std::weak_ptr<Face>>(2);
}

geometry::HalfEdge::HalfEdge (Vertex begPoint, Vertex endPoint, ptrFace leftFace, ptrFace rightFace) {
  point = std::vector<Vertex>{begPoint, endPoint};
  face = std::vector<std::weak_ptr<Face>>{leftFace, rightFace};
}

geometry::Face::Face(std::vector<Vertex> vertices) {
  vertex = vertices;
  numOfVertex = vertex.size();
  halfEdge.reserve(numOfVertex);

  normal = (vertex[1] - vertex[0]).cross(vertex[2] - vertex[0]);
  normal /= normal.norm();

  for (size_t i=0; i<numOfVertex; ++i) {
    halfEdge.emplace_back(HalfEdge(vertex[i], vertex[(i+1)%numOfVertex]));
  }
}

void geometry::Face::construct_halfEdge() {
  for (size_t i=0; i<numOfVertex; ++i) {
    halfEdge[i].face[0] = shared_from_this();
  }
}

geometry::Triangle::Triangle(std::vector<Vertex> vertices) : Face(std::vector<Vertex>(vertices.begin(), vertices.begin()+3)) {
}

