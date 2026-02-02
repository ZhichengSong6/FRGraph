#ifndef _FREE_REGIONS_GRAPH_H_
#define _FREE_REGIONS_GRAPH_H_

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include <decomp_ros_utils/data_ros_utils.h>

struct GraphNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // the position of the robot when planning process starts
    Eigen::Vector3d replan_pos_;
    Eigen::Matrix2d R_2d_;

    Polyhedron3D polys_;
    Polyhedron2D polys_2d_;

    bool root = false;
    bool visited = false;

    GraphNode *parent;
    std::vector<GraphNode*> children;
};

class FreeRegionsGraph {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::unique_ptr<FreeRegionsGraph> Ptr;

    FreeRegionsGraph() {}
    ~FreeRegionsGraph() {}

    void setRootNode(Eigen::Vector3d start_pos);
    GraphNode* getRootNode() { return root_; }
    GraphNode* getParentNode(GraphNode* node) { return node->parent; }

    private:
    GraphNode* root_;
};

#endif  // _FREE_REGIONS_GRAPH_H_