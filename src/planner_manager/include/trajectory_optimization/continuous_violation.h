# ifndef CONTINUOUS_VIOLATION_H
# define CONTINUOUS_VIOLATION_H

# include <Eigen/Dense>
# include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

# include "trajectory_optimization/trajectory.h"
# include "piqp/piqp.hpp"

struct WorstViolation2D {
  bool safe = false;
  double t = 0.0;
  int plane_k = -1;
  int vert_i = -1;
  double g = -1e100;
};

struct VerifyOptions {
  double eps = 1e-6;       // safety tolerance
  double min_dt = 1e-4;    // smallest interval length
  int max_nodes = 5000;    // cap on number of interval nodes expanded
  bool unit_normals = false; // if rows of A are already normalized
};

WorstViolation2D FindWorstViolationContinuous2D(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector2d>& robot_vertices,
    const BezierSE2& traj,
    const VerifyOptions& opt);

struct IntervalNode {
  double tL, tR;
  double U;              // upper bound on max violation in this interval
  WorstViolation2D mid;  // true max at midpoint
};

struct NodeCmp {
  bool operator()(const IntervalNode& a, const IntervalNode& b) const {
    return a.U < b.U; // max-heap by U
  }
};

struct ViolatedPlaneAtT {
  int k = -1;              // plane index
  int i = -1;              // active vertex index (support vertex)
  double g = 0.0;          // algebraic violation at t*
  Eigen::Vector2d a;       // plane normal
  double b = 0.0;          // plane offset
  double dgdtheta = 0.0;   // a^T R S v_i
};

std::vector<ViolatedPlaneAtT> collectViolatedPlanesAtT(
    double t_star,
    const Eigen::MatrixXd& A,        // m x 2
    const Eigen::VectorXd& b,        // m
    const std::vector<Eigen::Vector2d>& verts_body,
    const BezierSE2& traj,
    double eps_add,
    int topK = -1); // topK<0 => keep all

void simpleUpdatePosOnlyTopK(
    BezierSE2& traj,
    double t_star,
    const ViolatedPlaneAtT& worst_plane, // take violated_planes[0]
    double eta_pos,                      
    int Kcp); // for test

bool RepairOnce_PIQP(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const std::vector<Eigen::Vector2d>& robot_vertices,
    BezierSE2& traj,
    const VerifyOptions& opt,
    int Kcp,
    int topKplanes,
    double eps_add,
    double margin,
    double delta_p,
    double delta_th,
    double w_p,
    double w_th,
    double w_slack);
    
# endif // CONTINUOUS_VIOLATION_H