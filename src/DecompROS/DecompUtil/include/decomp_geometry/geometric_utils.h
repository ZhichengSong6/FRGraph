/**
 * @file geometric_utils.h
 * @brief basic geometry utils
 */
#ifndef DECOMP_GEOMETRIC_UTILS_H
#define DECOMP_GEOMETRIC_UTILS_H

#include <Eigen/Eigenvalues>
#include <decomp_basis/data_utils.h>
#include <decomp_geometry/polyhedron.h>
#include <iostream>

/// Calculate eigen values
template <int Dim> Vecf<Dim> eigen_value(const Matf<Dim, Dim> &A) {
  Eigen::SelfAdjointEigenSolver<Matf<Dim, Dim>> es(A);
  return es.eigenvalues();
}

/// Calculate rotation matrix from a vector (aligned with x-axis)
inline Mat2f vec2_to_rotation(const Vec2f &v) {
  decimal_t yaw = std::atan2(v(1), v(0));
  Mat2f R;
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  return R;
}

inline Mat3f vec3_to_rotation(const Vec3f &v) {
  // zero roll
  Vec3f rpy(0, std::atan2(-v(2), v.topRows<2>().norm()),
            std::atan2(v(1), v(0)));
  Quatf qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
  Quatf qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
  Quatf qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
  return Mat3f(qz * qy * qx);
}

/// Sort plannar points in the counter-clockwise order
inline vec_Vec2f sort_pts(const vec_Vec2f &pts) {
  /// if empty, dont sort
  if (pts.empty())
    return pts;
  /// calculate center point
  Vec2f avg = Vec2f::Zero();
  for (const auto &pt : pts)
    avg += pt;
  avg /= pts.size();

  /// sort in body frame
  vec_E<std::pair<decimal_t, Vec2f>> pts_valued;
  pts_valued.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    decimal_t theta = atan2(pts[i](1) - avg(1), pts[i](0) - avg(0));
    pts_valued[i] = std::make_pair(theta, pts[i]);
  }

  std::sort(
      pts_valued.begin(), pts_valued.end(),
      [](const std::pair<decimal_t, Vec2f> &i,
         const std::pair<decimal_t, Vec2f> &j) { return i.first < j.first; });
  vec_Vec2f pts_sorted(pts_valued.size());
  for (size_t i = 0; i < pts_valued.size(); i++)
    pts_sorted[i] = pts_valued[i].second;
  return pts_sorted;
}

/// Find intersection between two lines on the same plane, return false if they
/// are not intersected
inline bool line_intersect(const std::pair<Vec2f, Vec2f> &v1,
                           const std::pair<Vec2f, Vec2f> &v2, Vec2f &pi) {
  decimal_t a1 = -v1.first(1);
  decimal_t b1 = v1.first(0);
  decimal_t c1 = a1 * v1.second(0) + b1 * v1.second(1);

  decimal_t a2 = -v2.first(1);
  decimal_t b2 = v2.first(0);
  decimal_t c2 = a2 * v2.second(0) + b2 * v2.second(1);

  decimal_t x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  decimal_t y = (c1 * a2 - c2 * a1) / (a2 * b1 - a1 * b2);

  if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y))
    return false;
  else {
    pi << x, y;
    return true;
  }
}

/// Find intersection between multiple lines
inline vec_Vec2f line_intersects(const vec_E<std::pair<Vec2f, Vec2f>> &lines) {
  vec_Vec2f pts;
  for (unsigned int i = 0; i < lines.size(); i++) {
    for (unsigned int j = i + 1; j < lines.size(); j++) {
      Vec2f pi;
      if (line_intersect(lines[i], lines[j], pi)) {
        pts.push_back(pi);
      }
    }
  }
  return pts;
}

/// Find extreme points of Polyhedron2D
inline vec_Vec2f cal_vertices(const Polyhedron2D &poly) {
  vec_E<std::pair<Vec2f, Vec2f>> lines;
  const auto vs = poly.hyperplanes();
  for (unsigned int i = 0; i < vs.size(); i++) {
    Vec2f n = vs[i].n_;
    Vec2f v(-n(1), n(0));
    v = v.normalized();

    lines.push_back(std::make_pair(v, vs[i].p_));
    /*
    std::cout << "add p: " << lines.back().second.transpose() <<
      " v: " << lines.back().first.transpose() << std::endl;
      */
  }

  auto vts = line_intersects(lines);
  // for(const auto& it: vts)
  // std::cout << "vertice: " << it.transpose() << std::endl;

  vec_Vec2f vts_inside = poly.points_inside(vts);
  vts_inside = sort_pts(vts_inside);

  return vts_inside;
}

/// Find extreme points of Polyhedron3D
inline vec_E<vec_Vec3f> cal_vertices(const Polyhedron3D &poly) {
  vec_E<vec_Vec3f> bds;
  const auto vts = poly.hyperplanes();
  //**** for each plane, find lines on it
  for (unsigned int i = 0; i < vts.size(); i++) {
    const Vec3f t = vts[i].p_;
    const Vec3f n = vts[i].n_;
    const Quatf q = Quatf::FromTwoVectors(Vec3f(0, 0, 1), n);
    const Mat3f R(q); // body to world
    vec_E<std::pair<Vec2f, Vec2f>> lines;
    for (unsigned int j = 0; j < vts.size(); j++) {
      if (j == i)
        continue;
      Vec3f nw = vts[j].n_;
      Vec3f nb = R.transpose() * nw;
      decimal_t bb = vts[j].p_.dot(nw) - nw.dot(t);
      Vec2f v = Vec3f(0, 0, 1).cross(nb).topRows<2>(); // line direction
      Vec2f p;                                         // point on the line
      if (nb(1) != 0)
        p << 0, bb / nb(1);
      else if (nb(0) != 0)
        p << bb / nb(0), 0;
      else
        continue;
      lines.push_back(std::make_pair(v, p));
    }

    //**** find all intersect points
    vec_Vec2f pts = line_intersects(lines);
    //**** filter out points inside polytope
    vec_Vec2f pts_inside;
    for (const auto &it : pts) {
      Vec3f p = R * Vec3f(it(0), it(1), 0) + t; // convert to world frame
      if (poly.inside(p))
        pts_inside.push_back(it);
    }

    if (pts_inside.size() > 2) {
      //**** sort in plane frame
      pts_inside = sort_pts(pts_inside);

      //**** transform to world frame
      vec_Vec3f points_valid;
      for (auto &it : pts_inside)
        points_valid.push_back(R * Vec3f(it(0), it(1), 0) + t);

      //**** insert resulting polygon
      bds.push_back(points_valid);
    }
  }
  return bds;
}

/// Remove reduntant hyperplanes using polyhedron vertices (2D)
inline void remove_redundant_hyperplanes(Polyhedron2D &poly, decimal_t tol = 1e-6){
  const auto vertices = cal_vertices(poly);
  if (vertices.empty()){
    return;
  }
  vec_E<Hyperplane2D> kept;
  kept.reserve(poly.vs_.size());
  for (const auto& hp : poly.vs_){
    bool active = false;
    for (const auto& v : vertices){
      if (std::abs(hp.signed_dist(v)) < tol){
        active = true;
        break;
      }
    }
    if (active) {
      kept.push_back(hp);
    }
  }
  poly.vs_ = kept;
}

/// Remove reduntant hyperplanes using polyhedron vertices (3D)
inline void remove_redundant_hyperplanes(Polyhedron3D &poly, decimal_t tol = 1e-6){
  const auto faces = cal_vertices(poly);
  vec_Vec3f vertices;
  for (const auto& face : faces){
    for (const auto& v : face){
      vertices.push_back(v);
    }
  }
  if (vertices.empty()){
    return;
  }
  vec_E<Hyperplane3D> kept;
  kept.reserve(poly.vs_.size());
  for (const auto& hp : poly.vs_){
    bool active = false;
    for (const auto& v : vertices){
      if (std::abs(hp.signed_dist(v)) < tol){
        active = true;
        break;
      }
    }
    if (active) {
      kept.push_back(hp);
    }
  }
  poly.vs_ = kept;
}

inline void prune_near_duplicate_hyperplanes(Polyhedron2D &poly,
                                            const Vec2f &inside_pt,
                                            decimal_t angle_deg = 5.0,
                                            decimal_t b_thresh  = 2e-3) {
  if (poly.vs_.empty()) return;

  const double cos_th = std::cos((double)angle_deg * M_PI / 180.0);

  vec_E<Hyperplane2D> kept;
  kept.reserve(poly.vs_.size());

  std::vector<Vec2f> kept_A_unit;
  std::vector<double> kept_b_unit;
  kept_A_unit.reserve(poly.vs_.size());
  kept_b_unit.reserve(poly.vs_.size());

  for (const auto &hp : poly.vs_) {
    vec_E<Hyperplane2D> one;
    one.push_back(hp);

    LinearConstraint2D lc(inside_pt, one);

    Vec2f A_raw = lc.A_.row(0).transpose();
    const double nrm = (double)A_raw.norm();
    if (nrm < 1e-12) continue;

    const Vec2f A_unit = (float)(1.0 / nrm) * A_raw;
    const double b_unit = (double)lc.b()(0) / nrm;

    bool merged = false;
    for (size_t k = 0; k < kept.size(); ++k) {
      const double cosang = (double)A_unit.dot(kept_A_unit[k]);
      if (cosang > cos_th && std::abs(b_unit - kept_b_unit[k]) < (double)b_thresh) {
        if (b_unit < kept_b_unit[k]) {
          kept[k]         = hp;
          kept_A_unit[k]  = A_unit;
          kept_b_unit[k]  = b_unit;
        }
        merged = true;
        break;
      }
    }

    if (!merged) {
      kept.push_back(hp);
      kept_A_unit.push_back(A_unit);
      kept_b_unit.push_back(b_unit);
    }
  }

  poly.vs_ = kept;
}

/// Prune near-duplicate hyperplanes (3D)
inline void prune_near_duplicate_hyperplanes(Polyhedron3D &poly, const Vec3f &pt, decimal_t angle_deg = 5.0, decimal_t b_thresh = 1e-3){
  if (poly.vs_.size() < 2){
    return;
  }

  const double cos_th = std::cos((double)angle_deg * M_PI / 180.0);

  vec_E<Hyperplane3D> kept;
  kept.reserve(poly.vs_.size());

  std::vector<Vec3f> kept_A;
  std::vector<double> kept_b;
  kept_A.reserve(poly.vs_.size());
  kept_b.reserve(poly.vs_.size());

  for (const auto& hp : poly.vs_){
    vec_E<Hyperplane3D> one;
    one.push_back(hp);
    LinearConstraint3D lc(pt, one);
    Vec3f A_raw = lc.A_.row(0).transpose();
    const double nrm = A_raw.norm();

    const Vec3f A_unit = (float)(1.0 / nrm) * A_raw;
    const double b_unit = lc.b()(0) / nrm;

    bool merged = false;
    for (size_t k= 0; k < kept.size(); k++){
      // same orientation only
      const double cosang = (double)A_unit.dot(kept_A[k]);
      if (cosang > cos_th && std::abs(b_unit - kept_b[k]) < (double)b_thresh){
        // keep tighter constraint: smaller b
        if (b_unit < kept_b[k]){
          kept[k] = hp;
          kept_A[k] = A_unit;
          kept_b[k] = b_unit;
      }
        merged = true;
        break;
      }
    }
    if (!merged){
      kept.push_back(hp);
      kept_A.push_back(A_unit);
      kept_b.push_back(b_unit);
    }
  }
  poly.vs_ = kept;
} 

/// Get the convex hull of a 2D points array, use wrapping method
inline vec_Vec2f cal_convex_hull(const vec_Vec2f &pts) {
  /// find left most point
  Vec2f p0;
  decimal_t min_x = std::numeric_limits<decimal_t>::infinity();
  for (const auto &it : pts) {
    if (min_x > it(0) || (min_x == it(0) && it(1) < p0(1))) {
      min_x = it(0);
      p0 = it;
    }
  }

  vec_Vec2f vs;
  vs.push_back(p0);

  while (vs.back() != p0 || vs.size() == 1) {
    const auto ref_pt = vs.back();
    Vec2f end_pt = p0;
    for (size_t i = 0; i < pts.size(); i++) {
      if (pts[i] == ref_pt)
        continue;
      Vec2f dir = (pts[i] - ref_pt).normalized();
      Hyperplane2D hp(ref_pt, Vec2f(-dir(1), dir(0)));
      bool most_left_hp = true;
      for (size_t j = 0; j < pts.size(); j++) {
        if (hp.signed_dist(pts[j]) > 0 && pts[j] != pts[i] &&
            pts[j] != ref_pt) {
          // if(hp.signed_dist(pts[j]) > 0) {
          most_left_hp = false;
          break;
        }
      }

      if (most_left_hp) {
        end_pt = pts[i];
        break;
      }
    }
    // std::cout << "add: " << end_pt.transpose() << std::endl;
    vs.push_back(end_pt);
  }

  return vs;
}

inline Polyhedron2D get_convex_hull(const vec_Vec2f &pts) {
  Polyhedron2D poly;
  Vec2f prev_dir(-1, -1);
  for (size_t i = 0; i < pts.size() - 1; i++) {
    size_t j = i + 1;
    Vec2f dir = (pts[j] - pts[i]).normalized();
    if (dir != prev_dir) {
      poly.add(Hyperplane2D((pts[i] + pts[j]) / 2, Vec2f(-dir(1), dir(0))));
      prev_dir = dir;
    }
  }

  return poly;
}

/// Minkowski sum, add B to A with center Bc
inline Polyhedron2D minkowski_sum(const Polyhedron2D &A, const Polyhedron2D &B,
                                  const Vec2f &Bc) {
  const auto A_vertices = cal_vertices(A);
  const auto B_vertices = cal_vertices(B);

  vec_Vec2f C_vertices;
  for (const auto &it : A_vertices) {
    for (const auto &itt : B_vertices)
      C_vertices.push_back(it + itt - Bc);
  }

  return get_convex_hull(cal_convex_hull(C_vertices));
}

#endif
