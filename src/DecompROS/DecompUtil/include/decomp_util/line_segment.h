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

    void dilate_aniso_full(const Vecf<Dim> &center, const float radius){
      find_polyhedron_aniso_full(radius, center);
      add_local_bbox_aniso(this->polyhedron_);
// std::cout << "Aniso full polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
      prune_near_duplicate_hyperplanes(this->polyhedron_, p1_, 5.0, 1e-3);
// std::cout << "After pruning near-duplicate hyperplanes, aniso full polyhedron has " << this->polyhedron_.vs_.size() << " faces." << std::endl;
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

    Vecf<Dim> select_point(const Vecf<Dim> &center, const vec_Vecf<Dim> &obs){
      Vecf<Dim> pt = Vecf<Dim>::Zero();

      if (obs.empty()) {
        return pt;
      }

      const Vecf<Dim> e = (p2_ - p1_).normalized();

      // ---------- 1) first try front-half candidates ----------
      bool found_front = false;
      decimal_t best_front_score = std::numeric_limits<decimal_t>::max();

      for (const auto &it : obs) {
        const Vecf<Dim> d = it - center;
        const decimal_t s = d.dot(e);

        // front half: projection on forward direction is non-negative
        if (s >= 0) {
          const Vecf<Dim> lateral_vec = d - s * e;
          const decimal_t d_lat = lateral_vec.norm();

          if (d_lat < best_front_score) {
            best_front_score = d_lat;
            pt = it;
            found_front = true;
          }
        }
      }

      if (found_front) {
        return pt;
      }

      // ---------- 2) if no front-half point, use back-half candidates ----------
      decimal_t best_back_score = std::numeric_limits<decimal_t>::max();

      for (const auto &it : obs) {
        const decimal_t d_center = (it - center).norm();

        if (d_center < best_back_score) {
          best_back_score = d_center;
          pt = it;
        }
      }

      return pt;
    }

    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      get_hyperplane(Vec2f obstacle_pt, Vec2f center, Hyperplane2D &hp, double radius) {
        Eigen::Vector2d seed_center = center;
        double eps = 1e-4;

        Eigen::VectorXd e = (p2_ - p1_).normalized();  
        Eigen::VectorXd obs_pt = obstacle_pt;          
        Eigen::VectorXd p1 = p1_;                      

        // lateral distance to guide line
        Eigen::VectorXd v = obs_pt - p1;
        double s = v.dot(e);
        Eigen::VectorXd rvec = v - s * e;
        double d_lat = rvec.norm();

        // switch threshold
        double w_max = 1.0;
        double s_pos = std::max(0.0, s);

        // --------- d_sw schedule (near -> far) ----------
        const double d_sw_far  = 0.8 * radius;          
        const double d_sw_near = 1.0 * radius;          
        const double s0        = 2.0 * radius;          

        double alpha = 1.0 - std::min(1.0, s_pos / s0); 
        alpha = alpha * alpha;                           

        double d_sw = d_sw_far + alpha * (d_sw_near - d_sw_far);
        // weight schedule
        double w = 0.0;
        if (d_lat < d_sw) {
          double t = 1.0 - d_lat / d_sw;   // in (0,1]
          w = w_max * t ;
        }

        // build Q
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Dim, Dim);
        Eigen::MatrixXd eeT = e * e.transpose();
        Eigen::MatrixXd Q = 2.0 * ( eeT + w * (I - eeT) );

        const double reg = 1e-8;
        Q += 2.0 * reg * I;

        // linear term c
        Eigen::VectorXd c = Eigen::VectorXd::Zero(Dim);

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
          std::cout << "[Free Space generation] In get_hyperplane(2d) PIQP failed to solve the QP and the status is " << static_cast<int>(result) << std::endl;
          return;
        }
        Hyperplane2D hp_tmp(obstacle_pt, Vec2f(sol(0), sol(1)));
        hp = hp_tmp;
      }

    template<int U = Dim>
      typename std::enable_if<U == 3>::type
      get_hyperplane(Vec3f obstacle_pt, Vec3f center, Hyperplane3D &hp, double radius) {
        Eigen::Vector3d seed_center = center;
        double eps = 1e-4;
        Eigen::VectorXd e = (p2_ - p1_).normalized();  
        Eigen::VectorXd obs_pt = obstacle_pt;          
        Eigen::VectorXd p1 = p1_;                      

        // lateral distance to guide line
        Eigen::VectorXd v = obs_pt - p1;
        double s = v.dot(e);
        Eigen::VectorXd rvec = v - s * e;
        double d_lat = rvec.norm();

        // switch threshold
        double w_max = 1.0;
        double s_pos = std::max(0.0, s);

        // --------- d_sw schedule (near -> far) ----------
        const double d_sw_far  = 0.8 * radius;          
        const double d_sw_near = 1.2 * radius;          
        const double s0        = 1.5 * radius;          

        double alpha = 1.0 - std::min(1.0, s_pos / s0); 
        alpha = alpha * alpha;                           

        double d_sw = d_sw_far + alpha * (d_sw_near - d_sw_far);
        // weight schedule
        double w = 0.0;
        if (d_lat < d_sw) {
          double t = 1.0 - d_lat / d_sw;   // in (0,1]
          w = w_max * t * t;
        }

        // build Q
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Dim, Dim);
        Eigen::MatrixXd eeT = e * e.transpose();
        Eigen::MatrixXd Q = 2.0 * ( eeT + w * (I - eeT) );

        const double reg = 1e-8;
        Q += 2.0 * reg * I;

        // linear term c
        Eigen::VectorXd c = Eigen::VectorXd::Zero(Dim);
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
          std::cout << "[Free Space generation] In get_hyperplane(3d) PIQP failed to solve the QP and the status is " << static_cast<int>(result) << std::endl;
          return;
        }
        Hyperplane3D hp_tmp(obstacle_pt, Vec3f(sol(0), sol(1), sol(2)));
        hp = hp_tmp;
      }

    template<int U = Dim>
    typename std::enable_if<U == 2>::type
    find_polyhedron_aniso_full(double radius, const Vecf<Dim> &center) {
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = this->obs_;

      if (obs_remain.empty()) {
        return;
      }

      while (!obs_remain.empty()) {
        // unified point selection over all remaining obstacles
        const auto pw = select_point(center, obs_remain);

        // generate hyperplane directly by QP
        Hyperplane2D v_qp(Vec2f::Zero(), Vec2f::Zero());
        get_hyperplane(pw, center, v_qp, radius);

        vec_E<Hyperplane2D> hp_vec;
        hp_vec.push_back(v_qp);
        LinearConstraint2D lc(center, hp_vec);

        Vec2f A_raw = lc.A_.row(0);
        double nrm = A_raw.norm();
        if (nrm < 1e-12) {
          std::cout << "[Free Space generation] In find_polyhedron_aniso_full(2d), hyperplane normal norm is too small." << std::endl;
          break;
        }

        Vec2f A = A_raw / nrm;
        double b0 = lc.b()(0) / nrm;

        const double eps_robot = 1e-4;
        const double eps_obs   = 1e-4;
        double b = tighten_b(A, b0, pw, obs_remain, eps_robot, eps_obs, 0.01);

        Hyperplane2D v(Vec2f::Zero(), Vec2f::Zero());
        v.n_ = A;
        v.p_ = A * b;

        Vs.add(v);

        vec_Vecf<Dim> obs_tmp;
        obs_tmp.reserve(obs_remain.size());
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < -1e-10)
            obs_tmp.push_back(it);
        }
        obs_remain.swap(obs_tmp);
      }

      if (Vs.vs_.empty()) {
        return;
      }

      this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(),
                                  Vs.vs_.begin(), Vs.vs_.end());
    }
    
    template<int U = Dim>
    typename std::enable_if<U == 3>::type
    find_polyhedron_aniso_full(double radius, const Vecf<Dim> &center) {
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = this->obs_;

      if (obs_remain.empty()) {
        return;
      }

      while (!obs_remain.empty()) {
        // unified point selection over all remaining obstacles
        const auto pw = select_point(center, obs_remain);

        // generate hyperplane directly by QP
        Hyperplane3D v_qp(Vec3f::Zero(), Vec3f::Zero());
        get_hyperplane(pw, center, v_qp, radius);

        vec_E<Hyperplane3D> hp_vec;
        hp_vec.push_back(v_qp);
        LinearConstraint3D lc(center, hp_vec);

        Vec3f A_raw = lc.A_.row(0);
        double nrm = A_raw.norm();
        if (nrm < 1e-12) {
          std::cout << "[Free Space generation] In find_polyhedron_aniso_full(3d), hyperplane normal norm is too small." << std::endl;
          break;
        }

        Vec3f A = A_raw / nrm;
        double b0 = lc.b()(0) / nrm;

        const double eps_robot = 1e-4;
        const double eps_obs   = 1e-4;
        double b = tighten_b(A, b0, pw, obs_remain, eps_robot, eps_obs, 0.01);

        Hyperplane3D v(Vec3f::Zero(), Vec3f::Zero());
        v.n_ = A;
        v.p_ = A * b;

        Vs.add(v);

        vec_Vecf<Dim> obs_tmp;
        obs_tmp.reserve(obs_remain.size());
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < -1e-10)
            obs_tmp.push_back(it);
        }
        obs_remain.swap(obs_tmp);
      }

      if (Vs.vs_.empty()) {
        return;
      }

      this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(),
                                  Vs.vs_.begin(), Vs.vs_.end());
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
