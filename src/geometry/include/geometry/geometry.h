#pragma once

#include <eigen3/Eigen/Dense>
#include <memory>

namespace geometry {
class HalfEdge;
class Face;

typedef Eigen::Vector3d Vertex;
typedef std::shared_ptr<Face> ptrFace;

class HalfEdge {
public:
// 有序顶点
std::vector<Vertex> point;
// 有序的相临面
std::vector<std::weak_ptr<Face>> face = std::vector<std::weak_ptr<Face>>(2);

public:
HalfEdge(Vertex begPoint, Vertex endPoint);
// HalfEdge(Vertex begPoint, Vertex endPoint, ptrFace leftFace);
HalfEdge(Vertex begPoint, Vertex endPoint, ptrFace leftFace, ptrFace rightFace);
}; // class HalfEdge

class Face : public std::enable_shared_from_this<Face> {
private:
size_t numOfVertex;

public:
std::vector<Vertex> vertex;
std::vector<HalfEdge> halfEdge;
Eigen::Vector3d normal;

public:
// Face(Face&& face);
Face(std::vector<Vertex> vertices);
void construct_halfEdge();

}; // class Face

class Triangle : public Face{
public:

public:
Triangle(std::vector<Vertex> vertices);
}; // class Triangle

} // namespace geometry
