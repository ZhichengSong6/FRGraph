/**
 * @file line_segment.h
 * @brief LineSegment Class
 */
#ifndef LINE_SEGMENT_H
#define LINE_SEGMENT_H

#include <decomp_util/decomp_base.h>
#include <decomp_geometry/geometric_utils.h>

#include <limits>
#include "piqp/piqp.hpp"

/**
 * @brief Line Segment Class
 *
 * The basic element in EllipsoidDecomp
 */
template <int Dim>
class LineSegment : public DecompBase<Dim> {
  public:
    ///Simple constructor
    LineSegment() {};
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    LineSegment(const Vecf<Dim> &p1, const Vecf<Dim> &p2) : p1_(p1), p2_(p2) {}
    /**
     * @brief Infalte the line segment
     * @param radius the offset added to the long semi-axis
     */
    void dilate(decimal_t radius) {
      find_ellipsoid(radius);
      this->find_polyhedron();
      add_local_bbox(this->polyhedron_);
    }

    void dilate_aniso(const Vecf<Dim> &center, const float radius){
      set_ellipsoid(center, radius);
      find_polyhedron_for_seed(center, radius);
      obs_pruning(center, radius);
      find_polyhedron_aniso(radius);
      add_local_bbox_aniso(this->polyhedron_);
      // std::cout << "Aniso polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
      remove_redundant_hyperplanes(this->polyhedron_);
      // std::cout << "After removing redundant hyperplanes, aniso polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
    }

    void dilate_aniso_full(const Vecf<Dim> &center, const float radius){
      set_ellipsoid(center, radius);
      find_polyhedron_for_seed(center, radius);
      obs_pruning(center, radius);
      find_polyhedron_aniso_full(radius, center);
      add_local_bbox_aniso(this->polyhedron_);
// std::cout << "Aniso full polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
      remove_redundant_hyperplanes(this->polyhedron_);
// std::cout << "After removing redundant hyperplanes, aniso full polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
    }

    /// Get the line
    vec_Vecf<Dim> get_line_segment() const {
      vec_Vecf<Dim> line;
      line.push_back(p1_);
      line.push_back(p2_);
      return line;
    }

    // set robot shape points
    void set_robot_shape_pts(const std::vector<Vecf<Dim>> &robot_shape_pts){
      robot_shape_pts_ = robot_shape_pts;

      const int Nr = (int)robot_shape_pts_.size();
      robot_pts_mat_.resize(Nr, Dim);
      for(int i = 0; i < Nr; i++){
        for(int k = 0; k < Dim; k++){
          robot_pts_mat_(i, k) = robot_shape_pts_[i](k);
        }
      }
      robot_pts_cached_ = true;
    }

  protected:
    ///Add the bounding box
    void add_local_bbox(Polyhedron<Dim> &Vs) {
      if(this->local_bbox_.norm() == 0)
        return;
      //**** virtual walls parallel to path p1->p2
      Vecf<Dim> dir = (p2_ - p1_).normalized();
      Vecf<Dim> dir_h = Vecf<Dim>::Zero();
      dir_h(0) = dir(1), dir_h(1) = -dir(0);
      if (dir_h.norm() == 0) {
        if(Dim == 2)
          dir_h << -1, 0;
        else
          dir_h << -1, 0, 0;
      }
      dir_h = dir_h.normalized();

      // along x
      Vecf<Dim> pp1 = p1_ + dir_h * this->local_bbox_(1);
      Vecf<Dim> pp2 = p1_ - dir_h * this->local_bbox_(1);
      Vs.add(Hyperplane<Dim>(pp1, dir_h));
      Vs.add(Hyperplane<Dim>(pp2, -dir_h));

      // along y
      Vecf<Dim> pp3 = p2_ + dir * this->local_bbox_(0);
      Vecf<Dim> pp4 = p1_ - dir * this->local_bbox_(0);
      Vs.add(Hyperplane<Dim>(pp3, dir));
      Vs.add(Hyperplane<Dim>(pp4, -dir));

      // along z
      if(Dim > 2) {
        Vecf<Dim> dir_v;
        dir_v(0) = dir(1) * dir_h(2) - dir(2) * dir_h(1);
        dir_v(1) = dir(2) * dir_h(0) - dir(0) * dir_h(2);
        dir_v(2) = dir(0) * dir_h(1) - dir(1) * dir_h(0);
        Vecf<Dim> pp5 = p1_ + dir_v * this->local_bbox_(2);
        Vecf<Dim> pp6 = p1_ - dir_v * this->local_bbox_(2);
        Vs.add(Hyperplane<Dim>(pp5, dir_v));
        Vs.add(Hyperplane<Dim>(pp6, -dir_v));
      }
    }

    // Add the bounding box anisotropically
    void add_local_bbox_aniso(Polyhedron<Dim> &Vs){
      if(this->local_bbox_aniso_pos.norm() == 0 &&
         this->local_bbox_aniso_neg.norm() == 0)
        return;
      //**** virtual walls parallel to path p1->p2
      Vecf<Dim> dir = (p2_ - p1_).normalized();
      Vecf<Dim> dir_h = Vecf<Dim>::Zero();
      dir_h(0) = dir(1), dir_h(1) = -dir(0);
      if (dir_h.norm() == 0) {
        if(Dim == 2)
          dir_h << -1, 0;
        else
          dir_h << -1, 0, 0;
      }
      dir_h = dir_h.normalized();

      // along x
      Vecf<Dim> pp1 = p1_ + dir_h * this->local_bbox_aniso_pos(1);
      Vecf<Dim> pp2 = p1_ - dir_h * this->local_bbox_aniso_neg(1);
      Vs.add(Hyperplane<Dim>(pp1, dir_h));
      Vs.add(Hyperplane<Dim>(pp2, -dir_h));

      // along y
      Vecf<Dim> pp3 = p2_ + dir * this->local_bbox_aniso_pos(0);
      Vecf<Dim> pp4 = p1_ - dir * this->local_bbox_aniso_neg(0);
      Vs.add(Hyperplane<Dim>(pp3, dir));
      Vs.add(Hyperplane<Dim>(pp4, -dir));

      // along z
      if(Dim > 2) {
        Vecf<Dim> dir_v;
        dir_v(0) = dir(1) * dir_h(2) - dir(2) * dir_h(1);
        dir_v(1) = dir(2) * dir_h(0) - dir(0) * dir_h(2);
        dir_v(2) = dir(0) * dir_h(1) - dir(1) * dir_h(0);
        Vecf<Dim> pp5 = p1_ + dir_v * this->local_bbox_aniso_pos(2);
        Vecf<Dim> pp6 = p1_ - dir_v * this->local_bbox_aniso_neg(2);
        Vs.add(Hyperplane<Dim>(pp5, dir_v));
        Vs.add(Hyperplane<Dim>(pp6, -dir_v));
      }
    }

    /// Find ellipsoid in 2D
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      find_ellipsoid(double offset_x) {
        const decimal_t f = (p1_ - p2_).norm() / 2;
        Matf<Dim, Dim> C = f * Matf<Dim, Dim>::Identity();
        Vecf<Dim> axes = Vecf<Dim>::Constant(f);
        C(0, 0) += offset_x;
        axes(0) += offset_x;

        if(axes(0) > 0) {
          double ratio = axes(1) / axes(0);
          axes *= ratio;
          C *= ratio;
        }

        const auto Ri = vec2_to_rotation(p2_ - p1_);
        C = Ri * C * Ri.transpose();

        Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);

        auto obs = E.points_inside(this->obs_);

        auto obs_inside = obs;
        //**** decide short axes
        while (!obs_inside.empty()) {
          const auto pw = E.closest_point(obs_inside);
          Vecf<Dim> p = Ri.transpose() * (pw - E.d()); // to ellipsoid frame
          if(p(0) < axes(0))
            axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
          Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
          new_C(0, 0) = axes(0);
          new_C(1, 1) = axes(1);
          E.C_ = Ri * new_C * Ri.transpose();
          E.update_cache();

          vec_Vecf<Dim> obs_new;
          for(const auto &it: obs_inside) {
            if(1 - E.dist(it) > epsilon_)
              obs_new.push_back(it);
          }
          obs_inside = obs_new;
        }

        this->ellipsoid_ = E;
      }

    /// Find ellipsoid in 3D
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      find_ellipsoid(double offset_x) {
      const decimal_t f = (p1_ - p2_).norm() / 2;
      Matf<Dim, Dim> C = f * Matf<Dim, Dim>::Identity();
      Vecf<Dim> axes = Vecf<Dim>::Constant(f);
      C(0, 0) += offset_x;
      axes(0) += offset_x;

      if(axes(0) > 0) {
        double ratio = axes(1) / axes(0);
        axes *= ratio;
        C *= ratio;
      }

      const auto Ri = vec3_to_rotation(p2_ - p1_);
      C = Ri * C * Ri.transpose();

      Ellipsoid<Dim> E(C, (p1_ + p2_) / 2);
      auto Rf = Ri;

      auto obs = E.points_inside(this->obs_);
      auto obs_inside = obs;
      //**** decide short axes
      while (!obs_inside.empty()) {
        const auto pw = E.closest_point(obs_inside);
        Vecf<Dim> p = Ri.transpose() * (pw - E.d()); // to ellipsoid frame
        const decimal_t roll = atan2(p(2), p(1));
        Rf = Ri * Quatf(cos(roll / 2), sin(roll / 2), 0, 0);
        p = Rf.transpose() * (pw - E.d());

        if(p(0) < axes(0))
          axes(1) = std::abs(p(1)) / std::sqrt(1 - std::pow(p(0) / axes(0), 2));
        Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
        new_C(0, 0) = axes(0);
        new_C(1, 1) = axes(1);
        new_C(2, 2) = axes(1);
        E.C_ = Rf * new_C * Rf.transpose();
        E.update_cache();

        vec_Vecf<Dim> obs_new;
        for(const auto &it: obs_inside) {
          if(1 - E.dist(it) > epsilon_)
            obs_new.push_back(it);
        }
        obs_inside = obs_new;
      }

      //**** reset ellipsoid with old axes(2)
      C = f * Matf<Dim, Dim>::Identity();
      C(0, 0) = axes(0);
      C(1, 1) = axes(1);
      C(2, 2) = axes(2);
      E.C_ = Rf * C * Rf.transpose();
      E.update_cache();

      obs_inside = E.points_inside(obs);

      while (!obs_inside.empty()) {
        const auto pw = E.closest_point(obs_inside);
        Vec3f p = Rf.transpose() * (pw - E.d());
        decimal_t dd = 1 - std::pow(p(0) / axes(0), 2) -
          std::pow(p(1) / axes(1), 2);
        if(dd > epsilon_)
          axes(2) = std::abs(p(2)) / std::sqrt(dd);
        Matf<Dim, Dim> new_C = Matf<Dim, Dim>::Identity();
        new_C(0, 0) = axes(0);
        new_C(1, 1) = axes(1);
        new_C(2, 2) = axes(2);
        E.C_ = Rf * new_C * Rf.transpose();
        E.update_cache();

        vec_Vecf<Dim> obs_new;
        for(const auto &it: obs_inside) {
          if(1 - E.dist(it) > epsilon_)
            obs_new.push_back(it);
        }
        obs_inside = obs_new;
      }

      this->ellipsoid_ = E;
      this->aniso_ellipsoid_ = E;
    }

    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      set_ellipsoid(const Vecf<Dim> &center, const float radius){
        // the input is a circle, use it to set an ellipsoid
        Matf<Dim, Dim> C = radius * Matf<Dim, Dim>::Identity();
        Ellipsoid<Dim> E(C, center);
        this->aniso_ellipsoid_ = E;
      }
    
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      set_ellipsoid(const Vecf<Dim> &center, const float radius){
        // the input is a sphere, use it to set an ellipsoid
        Matf<Dim, Dim> C = radius * Matf<Dim, Dim>::Identity();
        this->aniso_ellipsoid_ = Ellipsoid<Dim>(C, center);
      }

    /// check whether there is obstacle inside the seed circle(2D)
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      find_polyhedron_for_seed(const Vecf<Dim> &center, const float radius) {
        Polyhedron<Dim> Vs;
        // first check whether there is obstacle inside the seed circle
        auto obs_inside = points_inside_seed(center, radius, this->obs_);
int counter = 0;
// Vec2f e = (p2_ - p1_).normalized();
        while (!obs_inside.empty()) {
          // find the closest point to the seed center
          const auto pw = closest_point_in_seed(center, obs_inside);
std::cout << "[seed] closest point to seed center: " << pw.transpose() << std::endl;
// std::cout << "[seed] (pw - center).dot(e): " << (pw - center).dot(e) << std::endl;
          // base on the closest point, find the hyperplane
          Hyperplane2D hp(Vec2f::Zero(), Vec2f::Zero());
          // get_hyperplane(pw, center, hp);
          get_hyperplane_seed(pw, center, hp, radius);
          
          vec_E<Hyperplane2D> hp_vec;
          hp_vec.push_back(hp);
          LinearConstraint2D lc(center, hp_vec);
          Vec2f A_raw = lc.A_.row(0);
          double nrm = A_raw.norm();
          Vec2f A = A_raw / nrm;
          double b0 = lc.b()(0) / nrm;
          const double eps_robot = 1e-4;  
          const double eps_obs = 1e-4;    
          double b = tighten_b(A, b0, pw, obs_inside, eps_robot, eps_obs, 0.01);
          hp.n_ = A;
          hp.p_ = A * b;

          Vs.add(hp);
std::cout << "[seed] hyperplane normal: " << hp.n_.transpose() << ", point: " << hp.p_.transpose() << std::endl;
counter++;
          // remove the points which are on the negative side of the hyperplane
          vec_Vecf<Dim> obs_new;
          obs_new.reserve(obs_inside.size());
// int size_before = obs_inside.size();
          for (const auto &it : obs_inside) {
            if (hp.signed_dist(it) < -1e-10)
              obs_new.push_back(it);
          }
          obs_inside.swap(obs_new);
// std::cout << "obs reduced from " << size_before << " to " << obs_inside.size() << std::endl;
        }
std::cout << "[seed] Aniso full polyhedron generated " << counter << " hyperplanes." << std::endl;
        this->polyhedron_ = Vs;
      }
    
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      find_polyhedron_for_seed(const Vecf<Dim> &center, const float radius) {
        Polyhedron<Dim> Vs;
        // first check whether there is obstacle inside the seed circle
        auto obs_inside = points_inside_seed(center, radius, this->obs_);
        while (!obs_inside.empty()) {
          // find the closest point to the seed center
          const auto pw = closest_point_in_seed(center, obs_inside);
          // base on the closest point, find the hyperplane
          Hyperplane3D hp(Vec3f::Zero(), Vec3f::Zero());
          get_hyperplane(pw, center, hp);

          vec_E<Hyperplane3D> hp_vec;
          hp_vec.push_back(hp);
          LinearConstraint3D lc(center, hp_vec);
          Vec3f A_raw = lc.A_.row(0);
          double nrm = A_raw.norm();
          Vec3f A = A_raw / nrm;
          double b0 = lc.b()(0) / nrm;
          const double eps_robot = 1e-4;  
          const double eps_obs = 1e-4;    
          double b = tighten_b(A, b0, pw, obs_inside, eps_robot, eps_obs, 0.01);
          hp.n_ = A;
          hp.p_ = A * b;
          Vs.add(hp);
          // remove the points which are on the negative side of the hyperplane
          vec_Vecf<Dim> obs_new;
          for (const auto &it : obs_inside) {
            if (hp.signed_dist(it) < -1e-10)
              obs_new.push_back(it);
          }
          obs_inside.swap(obs_new);
        }
        this->polyhedron_ = Vs;
      }

    double tighten_b(const Vecf<Dim> &A_unit, double b0, const Vecf<Dim> &pw, const vec_Vecf<Dim> &obs, double eps_robot, double eps_obs, double tau, double sd_tol = 1e-9){
      // robot safety
      double robot_max = -std::numeric_limits<double>::max();
      for(const auto &it : robot_shape_pts_){
        robot_max = std::max(robot_max, (double)(A_unit.dot(it)));
      }
      const double b_robot = robot_max + eps_robot;
      // near-violating
      double b_near = std::numeric_limits<double>::infinity();
      for(const auto &it : obs){
        const double sd = (double)A_unit.dot(it) - b0;
        if (sd < tau && sd > sd_tol){
          b_near = std::min(b_near, (double)A_unit.dot(it) - eps_obs);
        }
      }
      if (!std::isfinite(b_near)){
        // no near-violating points, do not tighten
        return b0;
      }

      double b = std::min(b0, b_near);

      const double b_pw = (double)A_unit.dot(pw);
      b = std::min(b, b_pw - 1e-10); // ensure pw is outside

      b = std::max(b, b_robot);

      if(b > b_pw){
        return b0;
      }
      return b;
    }

    vec_Vecf<Dim> points_inside_seed(const Vecf<Dim> &center, const float radius, const vec_Vecf<Dim> &obs) {
      vec_Vecf<Dim> new_obs;
      for (const auto &it : obs) {
        if ((it - center).norm() <= radius)
          new_obs.push_back(it);
      }
      return new_obs;
    }

    void obs_pruning(const Vecf<Dim> &center, const float radius){
      vec_Vecf<Dim> new_obs;
      for (const auto &it : this->obs_) {
        if ((it - center).norm() > radius)
          new_obs.push_back(it);
      }
      this->obs_ = new_obs;
    }

    Vecf<Dim> closest_point_in_seed(const Vecf<Dim> &center, const vec_Vecf<Dim> &obs){
      Vecf<Dim> pt = Vecf<Dim>::Zero();
      decimal_t min_dist = std::numeric_limits<decimal_t>::max();
      for (const auto &it : obs) {
        decimal_t d = (it - center).norm();
        if (d < min_dist) {
          min_dist = d;
          pt = it;
        }
      }
      return pt;
    }

    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      get_hyperplane(Vec2f obstacle_pt, Vec2f center, Hyperplane2D &hp) {
        Eigen::Vector2d e = (p2_ - p1_).normalized();
        std::vector<Eigen::Vector2d> robot_shape_pts_2d;
        for (const auto &pt : robot_shape_pts_){
          robot_shape_pts_2d.push_back(Eigen::Vector2d(pt(0), pt(1)));
        }
        Eigen::Vector2d obs_pt = obstacle_pt;
        Eigen::Vector2d seed_center = center;
        double eps = 1e-4;
        // Q matrix
        Eigen::Matrix2d Q = 2.0 * (e * e.transpose());
        // linear term c
        Eigen::Vector2d c = Eigen::Vector2d::Zero();

        const int Nr = robot_pts_mat_.rows();
        if(!qp_cache_init_){
          A_ineq_cache_.resize(Nr, Dim);
          u_ineq_cache_.resize(Nr);
          A_eq_cache_.resize(1, Dim);
          b_eq_cache_.resize(1);
          qp_cache_init_ = true;
        }
        // inequalitiy constraints A_ineq * x <= u_ineq

        for (int i = 0; i < Nr; i++){
          A_ineq_cache_.row(i) = (robot_pts_mat_.row(i).transpose() - obs_pt).transpose();
          u_ineq_cache_(i) = -eps;
        }
        // Equality constraints A_eq * x = b_eq
        A_eq_cache_.row(0) = (seed_center - obs_pt).transpose();
        b_eq_cache_(0) = -1.0;

        if (!solver_init_){
          solver_.setup(Q, c,
                        A_eq_cache_, b_eq_cache_,
                        A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                        piqp::nullopt, piqp::nullopt);
          solver_init_ = true;
        }
        else{
          solver_.update(Q, c,
                         A_eq_cache_, b_eq_cache_,
                         A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                         piqp::nullopt, piqp::nullopt);
        }

        const auto result = solver_.solve();
        Eigen::VectorXd sol;
        if (result == piqp::Status::PIQP_SOLVED){
          sol = solver_.result().x;
        }
        else{
          std::cout << "PIQP failed to solve the QP and the status is " << static_cast<int>(result) << std::endl;
          return;
        }
        Hyperplane2D hp_tmp(obstacle_pt, Vec2f(sol(0), sol(1)));
        hp = hp_tmp;
      }

    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      get_hyperplane(Vec3f obstacle_pt, Vec3f center, Hyperplane3D &hp) {
        Eigen::Vector3d e = (p2_ - p1_).normalized();
        std::vector<Eigen::Vector3d> robot_shape_pts_3d;
        for (const auto &pt : robot_shape_pts_){
            robot_shape_pts_3d.push_back(Eigen::Vector3d(pt(0), pt(1), pt(2)));
        }
        Eigen::Vector3d obs_pt = obstacle_pt;
        Eigen::Vector3d seed_center = center;
        double eps = 1e-4;
        // Q matrix
        Eigen::Matrix3d Q = 2.0 * (e * e.transpose());
        // linear term c
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        // inequalitiy constraints A_ineq * x <= u_ineq
        const int Nr = robot_pts_mat_.rows();
        if(!qp_cache_init_){
          A_ineq_cache_.resize(Nr, Dim);
          u_ineq_cache_.resize(Nr);
          A_eq_cache_.resize(1, Dim);
          b_eq_cache_.resize(1);
          qp_cache_init_ = true;
        }
        // inequalitiy constraints A_ineq * x <= u_ineq

        for (int i = 0; i < Nr; i++){
          A_ineq_cache_.row(i) = (robot_pts_mat_.row(i).transpose() - obs_pt).transpose();
          u_ineq_cache_(i) = -eps;
        }
        // Equality constraints A_eq * x = b_eq
        A_eq_cache_.row(0) = (seed_center - obs_pt).transpose();
        b_eq_cache_(0) = -1.0;

        if (!solver_init_){
          solver_.setup(Q, c,
                        A_eq_cache_, b_eq_cache_,
                        A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                        piqp::nullopt, piqp::nullopt);
          solver_init_ = true;
        }
        else{
          solver_.update(Q, c,
                         A_eq_cache_, b_eq_cache_,
                         A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                         piqp::nullopt, piqp::nullopt);
        }

        const auto result = solver_.solve();
        Eigen::VectorXd sol;
        if (result == piqp::Status::PIQP_SOLVED){
          sol = solver_.result().x;
        }
        else{
          std::cout << "PIQP failed to solve the QP and the status is " << static_cast<int>(result) << std::endl;
          return;
        }
        Hyperplane3D hp_tmp(obstacle_pt, Vec3f(sol(0), sol(1), sol(2)));
        hp = hp_tmp;
      }
    
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      get_hyperplane_seed(Vec2f obstacle_pt, Vec2f center, Hyperplane2D &hp, double radius){
        Eigen::Vector2d e = (p2_ - p1_).normalized();
        Eigen::Vector2d obs_pt = obstacle_pt;
        Eigen::Vector2d seed_center = center;

        Eigen::Vector2d a = obs_pt - seed_center;
        double a_norm = a.norm();

        double eps = 1e-4;

        Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
        Eigen::Vector2d c = Eigen::Vector2d::Zero();

        double signed_proj = a.dot(e);

        const double reg = 1e-8;
        if (signed_proj >= 0.1 * radius){
          Eigen::Vector2d a_hat = a / a_norm;
          Eigen::Matrix2d P_prep = Eigen::Matrix2d::Identity() - a_hat * a_hat.transpose();
          Q = 2.0 * (e * e.transpose()) + 3.0 * P_prep;
        }
        else{
          Eigen::Vector2d a_hat = a / a_norm;
          Eigen::Matrix2d P_prep = Eigen::Matrix2d::Identity() - a_hat * a_hat.transpose();
          Q = 2.0 * P_prep;
        }
        Q += 2.0 * reg * Eigen::Matrix2d::Identity();

        // ---------- Constraints ----------
        const int Nr = robot_pts_mat_.rows();
        if(!qp_cache_init_){
          A_ineq_cache_.resize(Nr, Dim);
          u_ineq_cache_.resize(Nr);
          A_eq_cache_.resize(1, Dim);
          b_eq_cache_.resize(1);
          qp_cache_init_ = true;
        }

        // robot points must be strictly on negative side of plane through obs_pt:
        // (r_i - obs)^T n <= -eps
        for (int i = 0; i < Nr; i++){
          A_ineq_cache_.row(i) = (robot_pts_mat_.row(i).transpose() - obs_pt).transpose();
          u_ineq_cache_(i) = -eps;
        }
      
        // scaling + sign fixing:
        // (center - obs)^T n = -1   <=>  a^T n = 1
        A_eq_cache_.row(0) = (seed_center - obs_pt).transpose();
        b_eq_cache_(0) = -1.0;

        // ---------- Solve ----------
        if (!solver_init_){
          solver_.setup(Q, c,
                        A_eq_cache_, b_eq_cache_,
                        A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                        piqp::nullopt, piqp::nullopt);
          solver_init_ = true;
        } else {
          solver_.update(Q, c,
                        A_eq_cache_, b_eq_cache_,
                        A_ineq_cache_, piqp::nullopt, u_ineq_cache_,
                        piqp::nullopt, piqp::nullopt);
        }

        const auto result = solver_.solve();
        Eigen::VectorXd sol;
        if (result == piqp::Status::PIQP_SOLVED){
          sol = solver_.result().x;
        } else {
          std::cout << "PIQP failed to solve the QP and the status is "
                    << static_cast<int>(result) << std::endl;
          return;
        }

        // sol is normal
        Hyperplane2D hp_tmp(obstacle_pt, Vec2f(sol(0), sol(1)));
        hp = hp_tmp;

      }

    /// Anisotropic polyhedron (2D)
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      find_polyhedron_aniso(double radius) {
        Polyhedron<Dim> Vs;
        vec_Vecf<Dim> obs_remain = this->obs_;
        // first check if there is any hyperplnae generate by find_polyhedron_for_seed
        if(!this->polyhedron_.vs_.empty()){
          obs_remain = this->polyhedron_.points_inside(obs_remain);
        }
        
        if (obs_remain.empty()){
          return;
        }

        // we inflate the ellipsoid anisotropically
        const double scale_long = 1.5;
        const double scale_lat = 1.01;
        Vecf<Dim> e0 = (p2_ - p1_).normalized(); 

        Vecf<Dim> e1;
        e1 << -e0(1), e0(0);
        e1.normalize();

        Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
        R.col(0) = e0;   // local x: along line
        R.col(1) = e1;   // local y: lateral

        while (!obs_remain.empty()){
          aniso_inflate_ellipsoid(scale_long, scale_lat, R, obs_remain);

          const auto v = this->aniso_ellipsoid_.closest_hyperplane(obs_remain);
          Vs.add(v);

          vec_Vecf<Dim> obs_tmp;
          obs_tmp.reserve(obs_remain.size());
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < -1e-10)
              obs_tmp.push_back(it);
          }
          obs_remain.swap(obs_tmp);
        }
        if (Vs.vs_.empty()){
          return;
        }
        this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(), Vs.vs_.begin(), Vs.vs_.end());
      }
    
    /// Anisotropic polyhedron (3D)
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      find_polyhedron_aniso(double radius) {
        Polyhedron<Dim> Vs;
        vec_Vecf<Dim> obs_remain = this->obs_;
        // first check if there is any hyperplnae generate by find_polyhedron_for_seed
        if(!this->polyhedron_.vs_.empty()){
          obs_remain = this->polyhedron_.points_inside(obs_remain);
        }
        
        if (obs_remain.empty()){
          return;
        }

        // we inflate the ellipsoid anisotropically
        const double scale_long = 1.5;
        const double scale_lat = 1.01;
        Vecf<Dim> e0 = (p2_ - p1_).normalized(); 
        Vecf<Dim> seed = Vecf<Dim>::UnitX();
        if (std::abs(e0.dot(seed)) > 0.9){
          seed = Vecf<Dim>::UnitY();
        }
        Vecf<Dim> e1 = e0.cross(seed).normalized();
        Vecf<Dim> e2 = e0.cross(e1).normalized();

        Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
        R.col(0) = e0;   // local x: along line
        R.col(1) = e1;   // local y: lateral
        R.col(2) = e2;   // local z: vertical
        
        while (!obs_remain.empty()){
          aniso_inflate_ellipsoid(scale_long, scale_lat, R, obs_remain);

          const auto v = this->aniso_ellipsoid_.closest_hyperplane(obs_remain);
          Vs.add(v);

          vec_Vecf<Dim> obs_tmp;
          obs_tmp.reserve(obs_remain.size());
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < -1e-10)
              obs_tmp.push_back(it);
          }
          obs_remain.swap(obs_tmp);
        }
        if (Vs.vs_.empty()){
          return;
        }
        this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(), Vs.vs_.begin(), Vs.vs_.end());
      }

    /// Anisotropic polyhedron (2D)
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      find_polyhedron_aniso_full(double radius, const Vecf<Dim> &center) {
        Polyhedron<Dim> Vs;
        vec_Vecf<Dim> obs_remain = this->obs_;
        // first check if there is any hyperplnae generate by find_polyhedron_for_seed
        bool seed_has_constraints = false;
        if(!this->polyhedron_.vs_.empty()){
          obs_remain = this->polyhedron_.points_inside(obs_remain);
          seed_has_constraints = true;
        }
        
        if (obs_remain.empty()){
          return;
        }

        // we inflate the ellipsoid anisotropically
        const double scale_long = 1.5;
        const double scale_lat = 1.01;
        Vecf<Dim> e0 = (p2_ - p1_).normalized(); 

        Vecf<Dim> e1;
        e1 << -e0(1), e0(0);
        e1.normalize();

        Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
        R.col(0) = e0;   // local x: along line
        R.col(1) = e1;   // local y: lateral

        Vec2f e = (p2_ - p1_).normalized();
int counter = 0;
        while (!obs_remain.empty()){
          aniso_inflate_ellipsoid(scale_long, scale_lat, R, obs_remain);
          const auto pw = this->aniso_ellipsoid_.closest_point(obs_remain);
          Hyperplane2D v(Vec2f::Zero(), Vec2f::Zero());
          v = this->aniso_ellipsoid_.closest_hyperplane(pw);
          // gate for whether use qp to solve hyperplane
          bool points_ahead = (pw - center).dot(e) > 0.1 * radius;                  // whether the point is ahead of the seed center
          bool points_within = (pw - p1_).dot(e) - (p2_ - p1_).dot(e) < 0;          // whether the point is within the line segment
          Vec2f e_perp = Vec2f(-e(1), e(0));
          bool lateral_offset = (std::abs((pw - center).dot(e_perp)) < radius) && ((pw - p1_).dot(e) - 0.8 * (p2_ - p1_).dot(e) > 0);      // whether the point has large lateral offset (also need to be not too close to p1_)

std::cout << "closest point to seed center: " << pw.transpose() << std::endl;
          if (seed_has_constraints && points_ahead && points_within && !lateral_offset){
            Vec2f n = v.n_;
            n.normalize();
            // check angle between n and e
            const double align = std::abs(n.dot(e));
            const double align_thresh = std::cos(30 * M_PI / 180.0);
            if(align > align_thresh){
// std::cout << "(pw - center).dot(e): " << (pw - center).dot(e) << std::endl;
              Hyperplane2D v_qp(Vec2f::Zero(), Vec2f::Zero());
              get_hyperplane(pw, center, v_qp);

              vec_E<Hyperplane2D> hp_vec;
              hp_vec.push_back(v_qp);
              LinearConstraint2D lc(center, hp_vec);
              Vec2f A_raw = lc.A_.row(0);
              double nrm = A_raw.norm();
              Vec2f A = A_raw / nrm;
              double b0 = lc.b()(0) / nrm;

              const double eps_robot = 1e-4;  
              const double eps_obs = 1e-4;    
              double b = tighten_b(A, b0, pw, obs_remain, eps_robot, eps_obs, 0.01);
              v.n_ = A;
              v.p_ = A * b;
            }
          }
std::cout << "hyperplane normal: " << v.n_.transpose() << ", point: " << v.p_.transpose() << std::endl;
          Vs.add(v);
counter++;
          vec_Vecf<Dim> obs_tmp;
          obs_tmp.reserve(obs_remain.size());
// int size_before = obs_remain.size();
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < -1e-10)
              obs_tmp.push_back(it);
          }
          obs_remain.swap(obs_tmp);
// std::cout << "obs reduced from " << size_before << " to " << obs_remain.size() << std::endl;
        }
std::cout << "Aniso full polyhedron generated " << counter << " hyperplanes." << std::endl;
        if (Vs.vs_.empty()){
          return;
        }
        this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(), Vs.vs_.begin(), Vs.vs_.end());
      }      
    
    /// Anisotropic polyhedron (3D)
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      find_polyhedron_aniso_full(double radius,  const Vecf<Dim> &center) {
        Polyhedron<Dim> Vs;
        vec_Vecf<Dim> obs_remain = this->obs_;
        // first check if there is any hyperplnae generate by find_polyhedron_for_seed
        bool seed_has_constraints = false;
        if(!this->polyhedron_.vs_.empty()){
          obs_remain = this->polyhedron_.points_inside(obs_remain);
          seed_has_constraints = true;
        }
        
        if (obs_remain.empty()){
          return;
        }

        // we inflate the ellipsoid anisotropically
        const double scale_long = 1.5;
        const double scale_lat = 1.01;
        Vecf<Dim> e0 = (p2_ - p1_).normalized(); 
        Vecf<Dim> seed = Vecf<Dim>::UnitX();
        if (std::abs(e0.dot(seed)) > 0.9){
          seed = Vecf<Dim>::UnitY();
        }
        Vecf<Dim> e1 = e0.cross(seed).normalized();
        Vecf<Dim> e2 = e0.cross(e1).normalized();

        Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
        R.col(0) = e0;   // local x: along line
        R.col(1) = e1;   // local y: lateral
        R.col(2) = e2;   // local z: vertical
        
        while (!obs_remain.empty()){
          aniso_inflate_ellipsoid(scale_long, scale_lat, R, obs_remain);

          
          Hyperplane3D v(Vec3f::Zero(), Vec3f::Zero());
          if(seed_has_constraints){
            const auto pw = this->aniso_ellipsoid_.closest_point(obs_remain);
            get_hyperplane(pw, center, v);

            vec_E<Hyperplane3D> hp_vec;
            hp_vec.push_back(v);
            LinearConstraint3D lc(center, hp_vec);
            Vec3f A_raw = lc.A_.row(0);
            double nrm = A_raw.norm();
            Vec3f A = A_raw / nrm;
            double b0 = lc.b()(0) / nrm;
            const double eps_robot = 1e-4;  
            const double eps_obs = 1e-4;    
            double b = tighten_b(A, b0, pw, obs_remain, eps_robot, eps_obs, 0.01);
            v.n_ = A;
            v.p_ = A * b;
          }
          else{
            v = this->aniso_ellipsoid_.closest_hyperplane(obs_remain);
          }

          Vs.add(v);

          vec_Vecf<Dim> obs_tmp;
          obs_tmp.reserve(obs_remain.size());
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < -1e-10)
              obs_tmp.push_back(it);
          }
          obs_remain.swap(obs_tmp);
        }
        if (Vs.vs_.empty()){
          return;
        }
        this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(), Vs.vs_.begin(), Vs.vs_.end());
      }

    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      aniso_inflate_ellipsoid(double scale_long, double scale_lat, Matf<Dim, Dim>& R, vec_Vecf<Dim>& obs) {
        Matf<Dim, Dim> C_world = this->aniso_ellipsoid_.C_;
        Matf<Dim, Dim> C_local = R.transpose() * C_world * R;
        // first inflate long axis
        C_local(0,0) *= scale_long;
        C_local(0,1) = 0;
        C_local(1,0) = 0;
        // check whether there is any obstacle inside the ellipsoid
        C_world = R * C_local * R.transpose();
        Ellipsoid<Dim> E(C_world, this->aniso_ellipsoid_.d_);
        vec_Vecf<Dim> obs_tmp;
        obs_tmp = E.points_inside(obs);
        if (obs_tmp.empty()){
          // now inflate the short axis
          C_local(1,1) *= scale_lat;
          C_world = R * C_local * R.transpose();
          E.C_ = C_world;
          E.update_cache();

          obs_tmp = E.points_inside(obs);
          if (obs_tmp.empty()){
            this->aniso_ellipsoid_.C_ = C_world;
            this->aniso_ellipsoid_.update_cache();
            return;
          }
          else{
            // find the closest point
            const auto pw = E.closest_point(obs_tmp);
            // update the short axis
            Vecf<Dim> p = R.transpose() * (pw - E.d()); // to ellipsoid frame
            const double t = 1.0 - (p(0) * p(0)) / (C_local(0,0) * C_local(0,0));
            double b = std::abs(p(1)) / std::sqrt(t);
            C_local(1,1) = b;
            C_world = R * C_local * R.transpose();
            this->aniso_ellipsoid_.C_ = C_world;
            this->aniso_ellipsoid_.update_cache();
            return;
          }
        }
        else{
          // find the closest point
          const auto pw = E.closest_point(obs_tmp);
          // update the long axis
          Vecf<Dim> p = R.transpose() * (pw - E.d()); // to ellipsoid frame
          const double t = 1.0 - (p(1) * p(1)) / (C_local(1,1) * C_local(1,1));
          double a = std::abs(p(0)) / std::sqrt(t);
          C_local(0,0) = a;
          C_world = R * C_local * R.transpose();
          this->aniso_ellipsoid_.C_ = C_world;
          this->aniso_ellipsoid_.update_cache();
          return;
        }
      }
    
    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      aniso_inflate_ellipsoid(double scale_long, double scale_lat, Matf<Dim, Dim>& R, vec_Vecf<Dim>& obs) {
        Matf<Dim, Dim> C_world = this->aniso_ellipsoid_.C_;
        Matf<Dim, Dim> C_local = R.transpose() * C_world * R;
        // first inflate long axis
        C_local(0,0) *= scale_long;
        C_local(0,1) = 0;
        C_local(1,0) = 0;
        C_local(0,2) = 0;
        C_local(2,0) = 0;
        // check whether there is any obstacle inside the ellipsoid
        C_world = R * C_local * R.transpose();
        Ellipsoid<Dim> E(C_world, this->aniso_ellipsoid_.d_);
        vec_Vecf<Dim> obs_tmp;
        obs_tmp = E.points_inside(obs);
        if (obs_tmp.empty()){
          // now inflate the short axes
          C_local(1,1) *= scale_lat;
          C_local(2,2) *= scale_lat;
          C_world = R * C_local * R.transpose();
          E.C_ = C_world;
          E.update_cache();
          obs_tmp = E.points_inside(obs);
          if (obs_tmp.empty()){
            this->aniso_ellipsoid_.C_ = C_world;
            this->aniso_ellipsoid_.update_cache();
            return;
          }
          else{
            // find the closest point
            const auto pw = E.closest_point(obs_tmp);
            // update the short axes
            Vecf<Dim> p = R.transpose() * (pw - E.d()); // to ellipsoid frame
            const double t = 1.0 - (p(0)* p(0)) / (C_local(0,0) * C_local(0,0));
            double b = std::sqrt((p(1) * p(1))  + (p(2) * p(2))) / std::sqrt(t);
            C_local(1,1) = b;
            C_local(2,2) = b;
            C_world = R * C_local * R.transpose();
            this->aniso_ellipsoid_.C_ = C_world;
            this->aniso_ellipsoid_.update_cache();
            return;
          }
        }
        else{
          // find the closest point
          const auto pw = E.closest_point(obs_tmp);
          // update the long axis
          Vecf<Dim> p = R.transpose() * (pw - E.d()); // to ellipsoid frame
          const double t = 1.0 - (p(1)* p(1)) / (C_local(1,1) * C_local(1,1))
                           - (p(2)* p(2)) / (C_local(2,2) * C_local(2,2));
          double a = std::abs(p(0)) / std::sqrt(t);
          C_local(0,0) = a;
          C_world = R * C_local * R.transpose();
          this->aniso_ellipsoid_.C_ = C_world;
          this->aniso_ellipsoid_.update_cache();
          return;
        }
      }

    /// One end of line segment, input
    Vecf<Dim> p1_;
    /// The other end of line segment, input
    Vecf<Dim> p2_;
    /// vertices of the robot
    std::vector<Vecf<Dim>> robot_shape_pts_;
    Eigen::MatrixXd robot_pts_mat_;   // Nr x Dim
    bool robot_pts_cached_ = false;

    Eigen::MatrixXd A_ineq_cache_;   // Nr x Dim
    Eigen::VectorXd u_ineq_cache_;   // Nr
    Eigen::Matrix<double, 1, Eigen::Dynamic> A_eq_cache_; // 1 x Dim
    Eigen::VectorXd b_eq_cache_;     // 1

    bool qp_cache_init_ = false;

    piqp::DenseSolver<double> solver_;
    bool solver_init_ = false;
};

typedef LineSegment<2> LineSegment2D;

typedef LineSegment<3> LineSegment3D;
#endif
