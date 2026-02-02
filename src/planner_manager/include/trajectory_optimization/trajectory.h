#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <algorithm>

struct BezierSE2{
    static constexpr int N = 5; // degree
    static constexpr int K = 6; // control points

    // Control points: P[i] in R^2, Theta[i] scalar
    std::array<Eigen::Vector2d, K> P;
    std::array<double, K> Theta;

    // ---------- helpers ----------
    static inline double wrapToPi(double a) {
        // map to (-pi, pi]
        a = std::fmod(a + M_PI, 2.0 * M_PI);
        if (a < 0) a += 2.0 * M_PI;
        return a - M_PI;
    }


    static inline std::array<double, K> bernstein5(double t) {
        t = std::max(0.0, std::min(1.0, t));

        const double u = 1.0 - t;

        // B_i^5(t) = C(5,i) t^i (1-t)^(5-i)
        // coefficients: 1, 5, 10, 10, 5, 1
        std::array<double, K> B;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;

        const double u2 = u * u;
        const double u3 = u2 * u;
        const double u4 = u3 * u;
        const double u5 = u4 * u;

        B[0] = u5;
        B[1] = 5.0 * t * u4;
        B[2] = 10.0 * t2 * u3;
        B[3] = 10.0 * t3 * u2;
        B[4] = 5.0 * t4 * u;
        B[5] = t5;
        return B;
    }

    static inline std::array<double, 5> bernstein4(double t) {
        // degree 4 basis for derivative formula
        t = std::max(0.0, std::min(1.0, t));

        const double u = 1.0 - t;

        // coefficients: 1, 4, 6, 4, 1
        std::array<double, 5> B;
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;

        const double u2 = u * u;
        const double u3 = u2 * u;
        const double u4 = u3 * u;

        B[0] = u4;
        B[1] = 4.0 * t * u3;
        B[2] = 6.0 * t2 * u2;
        B[3] = 4.0 * t3 * u;
        B[4] = t4;
        return B;
    }

    // ---------- evaluation ----------
    inline Eigen::Vector2d pos(double t) const {
        const auto B = bernstein5(t);
        Eigen::Vector2d p = Eigen::Vector2d::Zero();
        for (int i = 0; i < K; ++i) p += B[i] * P[i];
        return p;
    }

    inline double yaw(double t) const {
        const auto B = bernstein5(t);
        double th = 0.0;
        for (int i = 0; i < K; ++i) th += B[i] * Theta[i];
        return th;
    }

    inline Eigen::Matrix2d R(double t) const {
        const double th = yaw(t);
        const double c = std::cos(th), s = std::sin(th);
        Eigen::Matrix2d R;
        R << c, -s,
            s,  c;
        return R;
    }

      // ---------- derivatives ----------
    // For degree N=5 Bezier: p'(t) = 5 * sum_{i=0..4} B_i^4(t) * (P_{i+1} - P_i)
    inline Eigen::Vector2d dpos(double t) const {
        const auto B4 = bernstein4(t);
        Eigen::Vector2d dp = Eigen::Vector2d::Zero();
        for (int i = 0; i < 5; ++i) dp += B4[i] * (P[i+1] - P[i]);
        return 5.0 * dp;
    }

    inline double dyaw(double t) const {
        const auto B4 = bernstein4(t);
        double dth = 0.0;
        for (int i = 0; i < 5; ++i) dth += B4[i] * (Theta[i+1] - Theta[i]);
        return 5.0 * dth;
    }

    // ---------- initialization from endpoints ----------
    // 6 control points placed uniformly on the straight segment; yaw also linearly interpolated with shortest angle.
    static BezierSE2 initFromEndpoints(const Eigen::Vector2d& p0, double th0,
                                        const Eigen::Vector2d& p1, double th1) {
        BezierSE2 traj;
        const Eigen::Vector2d dp = p1 - p0;

        const double dth = wrapToPi(th1 - th0); // shortest-angle delta
        for (int i = 0; i < K; ++i) {
        const double s = double(i) / double(K - 1); // 0, 0.2, ..., 1.0
        traj.P[i] = p0 + s * dp;
        traj.Theta[i] = th0 + s * dth;
        }
        return traj;
    }
};

#endif // TRAJECTORY_H