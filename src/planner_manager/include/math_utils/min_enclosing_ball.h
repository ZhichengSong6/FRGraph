#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <random>
#include <limits>
#include <cmath>

// ==========================
// 2D: Minimum Enclosing Circle
// ==========================

struct Circle2D {
  Eigen::Vector2d center;
  double radius;
};

// Check if a point is inside the circle (with small tolerance)
inline bool contains(const Circle2D& c,
                     const Eigen::Vector2d& p,
                     double eps = 1e-9) {
  return (p - c.center).squaredNorm() <= c.radius * c.radius + eps;
}

inline bool containsAll(const Circle2D& c,
                        const std::vector<Eigen::Vector2d>& pts,
                        double eps = 1e-9) {
  for (const auto& p : pts) {
    if (!contains(c, p, eps))
      return false;
  }
  return true;
}

// Construct a circle from a single point
inline Circle2D circleFrom1(const Eigen::Vector2d& p) {
  Circle2D c;
  c.center = p;
  c.radius = 0.0;
  return c;
}

// Construct a circle from two points (using them as diameter endpoints)
inline Circle2D circleFrom2(const Eigen::Vector2d& a,
                            const Eigen::Vector2d& b) {
  Circle2D c;
  c.center = 0.5 * (a + b);
  c.radius = 0.5 * (a - b).norm();
  return c;
}

// Construct the circumcircle from three points (if collinear, return invalid and let caller handle)
inline Circle2D circleFrom3(const Eigen::Vector2d& A,
                            const Eigen::Vector2d& B,
                            const Eigen::Vector2d& C,
                            bool& ok) {
  ok = false;
  Circle2D c;

  double a_x = A.x(), a_y = A.y();
  double b_x = B.x(), b_y = B.y();
  double c_x = C.x(), c_y = C.y();

  double d = 2.0 * (a_x*(b_y - c_y) +
                    b_x*(c_y - a_y) +
                    c_x*(a_y - b_y));
  if (std::abs(d) < 1e-12) {
    // Points are almost collinear; circumcircle is not uniquely defined
    return c;
  }

  double a2 = a_x * a_x + a_y * a_y;
  double b2 = b_x * b_x + b_y * b_y;
  double c2 = c_x * c_x + c_y * c_y;

  double ux = (a2*(b_y - c_y) +
               b2*(c_y - a_y) +
               c2*(a_y - b_y)) / d;
  double uy = (a2*(c_x - b_x) +
               b2*(a_x - c_x) +
               c2*(b_x - a_x)) / d;

  c.center = Eigen::Vector2d(ux, uy);
  c.radius = (c.center - A).norm();
  ok = true;
  return c;
}

// Trivial minimum circle when |R| <= 3
inline Circle2D trivialCircle(const std::vector<Eigen::Vector2d>& R) {
  if (R.empty()) {
    Circle2D c;
    c.center = Eigen::Vector2d::Zero();
    c.radius = 0.0;
    return c;
  }
  if (R.size() == 1) {
    return circleFrom1(R[0]);
  }
  if (R.size() == 2) {
    return circleFrom2(R[0], R[1]);
  }
  // R.size() == 3
  bool ok = false;
  Circle2D c = circleFrom3(R[0], R[1], R[2], ok);
  if (ok) return c;

  // Degenerate collinear case: among all 2-point circles, pick the smallest that contains all three points
  Circle2D best;
  best.radius = std::numeric_limits<double>::infinity();
  std::vector<int> idx = {0,1,2};
  for (int i = 0; i < 3; ++i) {
    for (int j = i+1; j < 3; ++j) {
      Circle2D tmp = circleFrom2(R[i], R[j]);
      if (containsAll(tmp, R) && tmp.radius < best.radius) {
        best = tmp;
      }
    }
  }
  // In theory we must find something; as a fallback, use the first two points
  if (!std::isfinite(best.radius)) {
    best = circleFrom2(R[0], R[1]);
  }
  return best;
}

// Welzl recursion for 2D circle
inline Circle2D welzlCircle(std::vector<Eigen::Vector2d>& P,
                            std::vector<Eigen::Vector2d>& R,
                            int n) {
  if (n == 0 || R.size() == 3)
    return trivialCircle(R);

  Eigen::Vector2d p = P[n-1];
  Circle2D D = welzlCircle(P, R, n-1);

  if (contains(D, p))
    return D;

  R.push_back(p);
  Circle2D D2 = welzlCircle(P, R, n-1);
  R.pop_back();
  return D2;
}

// Public API: compute 2D minimum enclosing circle
inline Circle2D minimumEnclosingCircle2D(std::vector<Eigen::Vector2d> P) {
  if (P.empty()) {
    Circle2D c;
    c.center = Eigen::Vector2d::Zero();
    c.radius = 0.0;
    return c;
  }

  // Shuffle points to ensure expected linear complexity for Welzl's algorithm
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(P.begin(), P.end(), g);

  std::vector<Eigen::Vector2d> R;
  return welzlCircle(P, R, static_cast<int>(P.size()));
}


// ==========================
// 3D: Minimum Enclosing Sphere
// ==========================

struct Sphere3D {
  Eigen::Vector3d center;
  double radius;
};

inline bool contains(const Sphere3D& s,
                     const Eigen::Vector3d& p,
                     double eps = 1e-9) {
  return (p - s.center).squaredNorm() <= s.radius * s.radius + eps;
}

inline bool containsAll(const Sphere3D& s,
                        const std::vector<Eigen::Vector3d>& pts,
                        double eps = 1e-9) {
  for (const auto& p : pts) {
    if (!contains(s, p, eps))
      return false;
  }
  return true;
}

// Sphere from 1 point
inline Sphere3D sphereFrom1(const Eigen::Vector3d& p) {
  Sphere3D s;
  s.center = p;
  s.radius = 0.0;
  return s;
}

// Sphere from 2 points (using them as diameter endpoints)
inline Sphere3D sphereFrom2(const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b) {
  Sphere3D s;
  s.center = 0.5 * (a + b);
  s.radius = 0.5 * (a - b).norm();
  return s;
}

// Sphere from 3 points: circumcircle of 3 coplanar points lifted to a sphere
inline Sphere3D sphereFrom3(const Eigen::Vector3d& A,
                            const Eigen::Vector3d& B,
                            const Eigen::Vector3d& C,
                            bool& ok) {
  ok = false;
  Sphere3D s;

  // Construct a local plane basis: e1, e2
  Eigen::Vector3d e1 = (B - A);
  double len1 = e1.norm();
  if (len1 < 1e-12) return s;
  e1 /= len1;

  Eigen::Vector3d v = C - A;
  double dot = v.dot(e1);
  Eigen::Vector3d tmp = v - dot * e1;
  double len2 = tmp.norm();
  if (len2 < 1e-12) {
    // Points are nearly collinear; leave to upper-level logic
    return s;
  }
  Eigen::Vector3d e2 = tmp / len2;

  // 2D coordinates in basis {e1, e2}
  Eigen::Vector2d a2(0.0, 0.0);
  Eigen::Vector2d b2(len1, 0.0);
  Eigen::Vector2d c2(v.dot(e1), v.dot(e2));

  bool ok2 = false;
  Circle2D circ = circleFrom3(a2, b2, c2, ok2);
  if (!ok2) return s;

  // Map the 2D circumcenter back to 3D
  Eigen::Vector3d center3D = A + circ.center.x() * e1 + circ.center.y() * e2;

  s.center = center3D;
  s.radius = (s.center - A).norm();
  ok = true;
  return s;
}

// Sphere from 4 points: circumsphere (if non-degenerate; otherwise ok=false)
inline Sphere3D sphereFrom4(const Eigen::Vector3d& p1,
                            const Eigen::Vector3d& p2,
                            const Eigen::Vector3d& p3,
                            const Eigen::Vector3d& p4,
                            bool& ok) {
  ok = false;
  Sphere3D s;

  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d r1 = p2 - p1;
  Eigen::Vector3d r2 = p3 - p1;
  Eigen::Vector3d r3 = p4 - p1;

  A.row(0) = 2.0 * r1.transpose();
  A.row(1) = 2.0 * r2.transpose();
  A.row(2) = 2.0 * r3.transpose();

  b(0) = p2.squaredNorm() - p1.squaredNorm();
  b(1) = p3.squaredNorm() - p1.squaredNorm();
  b(2) = p4.squaredNorm() - p1.squaredNorm();

  if (std::abs(A.determinant()) < 1e-12) {
    // Degenerate case: four points almost coplanar or linearly dependent
    return s;
  }

  Eigen::Vector3d center = A.colPivHouseholderQr().solve(b);
  s.center = center;
  s.radius = (s.center - p1).norm();
  ok = true;
  return s;
}

// Trivial minimum sphere when |R| <= 4:
// enumerate all non-empty subsets S of R with |S| <= 4,
// compute the sphere defined by S, and pick the smallest that contains all of R.
inline Sphere3D trivialSphere(const std::vector<Eigen::Vector3d>& R) {
  Sphere3D best;
  best.center = Eigen::Vector3d::Zero();
  best.radius = std::numeric_limits<double>::infinity();

  const int m = static_cast<int>(R.size());
  if (m == 0) {
    best.radius = 0.0;
    return best;
  }

  // Enumerate subsets using bitmask (up to 4 points → at most 2^4 = 16 subsets)
  int total_masks = 1 << m;
  for (int mask = 1; mask < total_masks; ++mask) {
    // Count subset size
    int cnt = 0;
    for (int i = 0; i < m; ++i)
      if (mask & (1 << i)) ++cnt;

    if (cnt > 4) continue;

    std::vector<int> idx;
    for (int i = 0; i < m; ++i)
      if (mask & (1 << i)) idx.push_back(i);

    Sphere3D s;
    bool ok = true;

    if (cnt == 1) {
      s = sphereFrom1(R[idx[0]]);
    } else if (cnt == 2) {
      s = sphereFrom2(R[idx[0]], R[idx[1]]);
    } else if (cnt == 3) {
      ok = false;
      s = sphereFrom3(R[idx[0]], R[idx[1]], R[idx[2]], ok);
    } else if (cnt == 4) {
      ok = false;
      s = sphereFrom4(R[idx[0]], R[idx[1]], R[idx[2]], R[idx[3]], ok);
    }

    if (!ok) continue;

    if (!containsAll(s, R)) continue;

    if (s.radius < best.radius) {
      best = s;
    }
  }

  // In theory we should always find a valid sphere; as a fallback:
  if (!std::isfinite(best.radius)) {
    // Use a sphere of radius 0 centered at the first point
    best = sphereFrom1(R[0]);
  }

  return best;
}

// Welzl recursion for 3D sphere
inline Sphere3D welzlSphere(std::vector<Eigen::Vector3d>& P,
                            std::vector<Eigen::Vector3d>& R,
                            int n) {
  if (n == 0 || static_cast<int>(R.size()) == 4)
    return trivialSphere(R);

  Eigen::Vector3d p = P[n-1];
  Sphere3D B = welzlSphere(P, R, n-1);

  if (contains(B, p))
    return B;

  R.push_back(p);
  Sphere3D B2 = welzlSphere(P, R, n-1);
  R.pop_back();
  return B2;
}

// Public API: compute 3D minimum enclosing sphere
inline Sphere3D minimumEnclosingSphere3D(std::vector<Eigen::Vector3d> P) {
  Sphere3D s;
  if (P.empty()) {
    s.center = Eigen::Vector3d::Zero();
    s.radius = 0.0;
    return s;
  }

  // Shuffle points to randomize input order for Welzl's algorithm
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(P.begin(), P.end(), g);

  std::vector<Eigen::Vector3d> R;
  s = welzlSphere(P, R, static_cast<int>(P.size()));
  return s;
}
