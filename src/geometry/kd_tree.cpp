#include "kd_tree.h"

kdtree::Node::Node(Eigen::MatrixXd pnt) : point(pnt), dim(pnt.size()) {
}

