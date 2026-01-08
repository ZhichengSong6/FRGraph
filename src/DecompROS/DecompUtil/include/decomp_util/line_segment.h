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
      find_polyhedron_aniso(radius);
      add_local_bbox_aniso(this->polyhedron_);
    }

    void dilate_aniso_full(const Vecf<Dim> &center, const float radius){
      set_ellipsoid(center, radius);
      find_polyhedron_for_seed(center, radius);
      find_polyhedron_aniso_full(radius);
      add_local_bbox_aniso(this->polyhedron_);
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
        while (!obs_inside.empty()) {
          // find the closest point to the seed center
          const auto pw = closest_point_in_seed(center, obs_inside);
          // base on the closest point, find the hyperplane
          Hyperplane2D hp(Vec2f::Zero(), Vec2f::Zero());
          get_hyperplane(pw, center, hp);
          Vs.add(hp);
          // remove the points which are on the negative side of the hyperplane
          vec_Vecf<Dim> obs_new;
          for (const auto &it : obs_inside) {
            if (hp.signed_dist(it) < 0)
              obs_new.push_back(it);
          }
          obs_inside.swap(obs_new);
        }
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
          Vs.add(hp);
          // remove the points which are on the negative side of the hyperplane
          vec_Vecf<Dim> obs_new;
          for (const auto &it : obs_inside) {
            if (hp.signed_dist(it) < 0)
              obs_new.push_back(it);
          }
          obs_inside.swap(obs_new);
        }
        this->polyhedron_ = Vs;
      }

    vec_Vecf<Dim> points_inside_seed(const Vecf<Dim> &center, const float radius, const vec_Vecf<Dim> &obs) {
      vec_Vecf<Dim> new_obs;
      for (const auto &it : obs) {
        if ((it - center).norm() <= radius)
          new_obs.push_back(it);
      }
      return new_obs;
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
        // inequalitiy constraints A_ineq * x <= u_ineq
        int Nr = robot_shape_pts_2d.size();
        Eigen::MatrixXd A_ineq(Nr, 2);
        Eigen::VectorXd u_ineq(Nr);
        for (int i = 0; i < Nr; i++){
          Eigen::Vector2d diff = robot_shape_pts_2d[i] - obs_pt;
          A_ineq.row(i) = diff.transpose();
          u_ineq(i) = -eps;
        }
        // Equality constraints A_eq * x = b_eq
        Eigen::Matrix<double, 1, 2> A_eq;
        Eigen::VectorXd b_eq(1);
        Eigen::Vector2d diff_eq = seed_center - obs_pt;
        A_eq.row(0) = diff_eq.transpose();
        b_eq(0) = -1.0;

        piqp::DenseSolver<double> solver;
        solver.setup(Q, c, A_eq, b_eq, A_ineq, piqp::nullopt, u_ineq, piqp::nullopt, piqp::nullopt);
        const auto result = solver.solve();
        Eigen::VectorXd sol;
        if (result == piqp::Status::PIQP_SOLVED){
          sol = solver.result().x;
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
        int Nr = robot_shape_pts_3d.size();
        Eigen::MatrixXd A_ineq(Nr, 3);
        Eigen::VectorXd u_ineq(Nr);
        for (int i = 0; i < Nr; i++){
          Eigen::Vector3d diff = robot_shape_pts_3d[i] - obs_pt;
          A_ineq.row(i) = diff.transpose();
          u_ineq(i) = -eps; 
        }
        // Equality constraints A_eq * x = b_eq
        Eigen::Matrix<double, 1, 3> A_eq;
        Eigen::VectorXd b_eq(1);
        Eigen::Vector3d diff_eq = seed_center - obs_pt;
        A_eq.row(0) = diff_eq.transpose();
        b_eq(0) = -1.0;

        piqp::DenseSolver<double> solver;
        solver.setup(Q, c, A_eq, b_eq, A_ineq, piqp::nullopt, u_ineq, piqp::nullopt, piqp::nullopt);
        const auto result = solver.solve();
        Eigen::VectorXd sol;
        if (result == piqp::Status::PIQP_SOLVED){
          sol = solver.result().x;
        }
        else{
          std::cout << "PIQP failed to solve the QP and the status is " << static_cast<int>(result) << std::endl;
          return;
        }
        Hyperplane3D hp_tmp(obstacle_pt, Vec3f(sol(0), sol(1), sol(2)));
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
          vec_Vecf<Dim> obs_tmp;
          obs_tmp = this->polyhedron_.points_inside(obs_remain);
          obs_remain = obs_tmp;
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
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < 0)
              obs_tmp.push_back(it);
          }
          obs_remain = obs_tmp;
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
          vec_Vecf<Dim> obs_tmp;
          obs_tmp = this->polyhedron_.points_inside(obs_remain);
          obs_remain = obs_tmp;
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
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < 0)
              obs_tmp.push_back(it);
          }
          obs_remain = obs_tmp;
        }
        if (Vs.vs_.empty()){
          return;
        }
        this->polyhedron_.vs_.insert(this->polyhedron_.vs_.end(), Vs.vs_.begin(), Vs.vs_.end());
      }
    
    /// Anisotropic polyhedron (2D)
    template<int U = Dim>
      typename std::enable_if<U == 2>::type
      find_polyhedron_aniso_full(double radius) {
        Polyhedron<Dim> Vs;
        vec_Vecf<Dim> obs_remain = this->obs_;
        // first check if there is any hyperplnae generate by find_polyhedron_for_seed
        if(!this->polyhedron_.vs_.empty()){
          vec_Vecf<Dim> obs_tmp;
          obs_tmp = this->polyhedron_.points_inside(obs_remain);
          obs_remain = obs_tmp;
        }
        
        if (obs_remain.empty()){
          return;
        }

        // we inflate the ellipsoid anisotropically
        const double scale_long = 2.0;
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

          // const auto v = this->aniso_ellipsoid_.closest_hyperplane(obs_remain);
          Eigen::Vector2d e = (p2_ - p1_).normalized();
          const auto v = this->aniso_ellipsoid_.closest_hyperplane_aniso(obs_remain, e, 0.2);
          Vs.add(v);

          vec_Vecf<Dim> obs_tmp;
          for (const auto &it : obs_remain) {
            if (v.signed_dist(it) < 0)
              obs_tmp.push_back(it);
          }
          obs_remain = obs_tmp;
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
          obs_tmp = E.points_inside(obs);
          if (obs_tmp.empty()){
            this->aniso_ellipsoid_.C_ = C_world;
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
          obs_tmp = E.points_inside(obs);
          if (obs_tmp.empty()){
            this->aniso_ellipsoid_.C_ = C_world;
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
          return;
        }
      }

    /// One end of line segment, input
    Vecf<Dim> p1_;
    /// The other end of line segment, input
    Vecf<Dim> p2_;
    /// vertices of the robot
    std::vector<Vecf<Dim>> robot_shape_pts_;
};

typedef LineSegment<2> LineSegment2D;

typedef LineSegment<3> LineSegment3D;
#endif
