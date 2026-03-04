#include "free_regions_graph/free_regions_graph.h"

NodeId FreeRegionsGraph::createNode(){
    NodeId id = static_cast<NodeId>(nodes_.size());
    nodes_.emplace_back(std::make_unique<GraphNode>());
    nodes_.back()->id = id;
    nodes_.back()->edge_ids.clear();
    return id;
}

void FreeRegionsGraph::setRootNode(Eigen::Vector3d start_pos) {
    // root_ = new GraphNode();
    // root_->parent = nullptr;
    // root_->children.clear();
    // root_->replan_pos_ = start_pos;
    // root_->root = true;
    // root_->visited = true;
    root_id_ = createNode();

    auto* root = getNode(root_id_);
    root->parent_id_ = -1;
    root->children_ids.clear();
    root->edge_ids.clear();
    root->state_pos_ = start_pos;
    root->visited = true;
}

EdgeId FreeRegionsGraph::addEdge(NodeId from, NodeId to, const Eigen::Vector3d& dir) {
    if (from < 0 || to < 0) return -1;
    if (static_cast<size_t>(from) >= nodes_.size()) return -1;
    if (static_cast<size_t>(to) >= nodes_.size()) return -1;

    EdgeId eid = static_cast<EdgeId>(edges_.size());
    edges_.emplace_back(std::make_unique<GraphEdge>());
    auto* e = edges_.back().get();

    e->id = eid;
    e->from = from;
    e->to = to;
    e->goal = dir;

    nodes_[from]->edge_ids.push_back(eid);

    return eid;
}

GraphNode* FreeRegionsGraph::getNode(NodeId id) {
    if (id < 0 || static_cast<size_t>(id) >= nodes_.size()) return nullptr;
    return nodes_[id].get();
}

GraphEdge* FreeRegionsGraph::getEdge(EdgeId id) {
    if (id < 0 || static_cast<size_t>(id) >= edges_.size()) return nullptr;
    return edges_[id].get();
}

NodeId FreeRegionsGraph::upsertNode(const Eigen::Vector3d& state_pos,
                                const Polyhedron3D& local_poly_3d) 
{
    // distance coarse check
    std::vector<NodeId> candidates;
    candidates.reserve(nodes_.size());

    for (NodeId i = 0; i < static_cast<NodeId>(nodes_.size()); ++i) {
        auto* n = getNode(i);
        if (!n) continue;
        const double dist = (n->state_pos_ - state_pos).norm();
        if (dist < merge_radius_) {
            candidates.push_back(i);
        }
    }

    // polyhedron inside check
    for (NodeId cid : candidates) {
        auto* n = getNode(cid);
        if (!n) continue;
        bool cond1 = true;
        bool cond2 = true;

        cond1 = n->polys_.inside(state_pos);
        cond2 = local_poly_3d.inside(n->state_pos_);
        if (cond1 && cond2){
            // n->state_pos_ = state_pos;
            // n->polys_ = local_poly_3d;
            return cid;
        }
    }
    NodeId nid = createNode();
    auto* nn = getNode(nid);
    nn->state_pos_ = state_pos;
    nn->polys_ = local_poly_3d;
    nn->visited = false;
    nn->parent_id_ = -1;
    nn->children_ids.clear();
    nn->edge_ids.clear();
    return nid;
}

NodeId FreeRegionsGraph::upsertNode(const Eigen::Vector2d& state_pos,
                                const Polyhedron2D& local_poly_2d) 
{
    // distance coarse check
    std::vector<NodeId> candidates;
    candidates.reserve(nodes_.size());

    for (NodeId i = 0; i < static_cast<NodeId>(nodes_.size()); ++i) {
        auto* n = getNode(i);
        if (!n) continue;
        const double dist = (n->state_pos_.head<2>() - state_pos).norm();
        if (dist < merge_radius_) {
            candidates.push_back(i);
        }
    }

    // polyhedron inside check
    for (NodeId cid : candidates) {
        auto* n = getNode(cid);
        if (!n) continue;
        bool cond1 = true;
        bool cond2 = true;

        cond1 = n->polys_2d_.inside(state_pos);
        cond2 = local_poly_2d.inside(n->state_pos_.head<2>());
        if (cond1 && cond2){
            // n->state_pos_.head<2>() = state_pos;
            // n->state_pos_(2) = 0.0;
            // n->polys_2d_ = local_poly_2d;
            return cid;
        }
    }
    NodeId nid = createNode();
    auto* nn = getNode(nid);
    nn->state_pos_.head<2>() = state_pos;
    nn->state_pos_(2) = 0.0;
    nn->polys_2d_ = local_poly_2d;
    nn->visited = false;
    nn->parent_id_ = -1;
    nn->children_ids.clear();
    nn->edge_ids.clear();
    return nid;
}