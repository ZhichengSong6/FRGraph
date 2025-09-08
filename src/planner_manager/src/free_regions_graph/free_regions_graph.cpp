#include "free_regions_graph/free_regions_graph.h"

void FreeRegionsGraph::setRootNode(Eigen::Vector3d start_pos) {
    root_ = new GraphNode();
    root_->parent = nullptr;
    root_->children.clear();
    root_->replan_pos_ = start_pos;
}