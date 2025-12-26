/**
 * @file decomp_base.h
 * @brief Decomp Base Class
 */
#ifndef DECOMP_BASE_H
#define DECOMP_BASE_H

#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
//#include <decomp_geometry/geometry_utils.h>

/**
 * @brief Line Segment Class
 *
 * The basic element in EllipsoidDecomp
 */
template <int Dim>
class DecompBase {
  public:
    ///Null constructor
    DecompBase() {}
    /**
     * @brief Adding local bounding box around line seg
     * @param Dim Distance in corresponding axis
     *
     * This virtual bounding box is parallel to the line segment, the x,y,z axes are not w.r.t the world coordinate system, but instead, x-axis is parallel to the line, y-axis is perpendicular to the line and world z-axis, z-axis is perpendiculat to the line and y-axis
     */
    void set_local_bbox(const Vecf<Dim>& bbox) {
      local_bbox_ = bbox;
    }

    void set_local_bbox_aniso(const Vecf<Dim>& bbox_pos,
                              const Vecf<Dim>& bbox_neg) {
      local_bbox_aniso_pos =  bbox_pos;
      local_bbox_aniso_neg =  bbox_neg;
    }

    ///Import obstacle points
    void set_obs(const vec_Vecf<Dim> &obs) {
      // only consider points inside local bbox
      Polyhedron<Dim> vs;
      add_local_bbox(vs);
      obs_ = vs.points_inside(obs);
    }

    ///Get obstacel points
    vec_Vecf<Dim> get_obs() const { return obs_; }

    ///Get ellipsoid
    Ellipsoid<Dim> get_ellipsoid() const { return ellipsoid_; }

    ///Get polyhedron
    Polyhedron<Dim> get_polyhedron() const { return polyhedron_; }

    /**
     * @brief Inflate the line segment
     * @param radius the offset added to the long semi-axis
     */
    virtual void dilate(decimal_t radius = 0) = 0;

    /**
     * @brief Shrink the polyhedron
     * @param shrink_distance Shrink distance
     */
    virtual void shrink(double shrink_distance) {}
 protected:
    virtual void add_local_bbox(Polyhedron<Dim> &Vs) = 0;

    void find_polyhedron() {
      //**** find half-space
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = obs_;
      while (!obs_remain.empty()) {
        const auto v = ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v);
        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
        /*
           std::cout << "a: " << a.transpose() << std::endl;
           std::cout << "b: " << b << std::endl;
           */
      }

      polyhedron_ = Vs;
    }

    /// Anisotropic polyhedron (2D)
    template<int U = Dim>
    typename std::enable_if<U == 2>::type
    find_polyhedron_aniso(const Vecf<Dim> &p1, const Vecf<Dim> &p2) {
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = obs_;

      // for the first half-space, we use the original ellipsoid
      if (!obs_remain.empty()) {
        const auto v0 = ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v0);
        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v0.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
      }

      if (obs_remain.empty()) {
        polyhedron_ = Vs;
        return;
      }

      Vecf<Dim> e0 = (p2 - p1).normalized(); 

      Vecf<Dim> e1;
      e1 << -e0(1), e0(0);
      e1.normalize();

      Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
      R.col(0) = e0;   // local x: along line
      R.col(1) = e1;   // local y: lateral

      const double scale_long = 1.1;   
      const double scale_lat  = 1.05;  
      const double max_lat    = 1.0;   

      while (!obs_remain.empty()) {
        Matf<Dim, Dim> C_world = aniso_ellipsoid_.C();
        Matf<Dim, Dim> C_local = R.transpose() * C_world * R;

        C_local(0,0) *= scale_long;          // long direction
        C_local(1,1) *= scale_lat;           // lateral
        C_local(1,1) = std::min(max_lat, C_local(1,1));

        C_local(0,1) = C_local(1,0) = 0;

        C_world = R * C_local * R.transpose();
        aniso_ellipsoid_.C_ = C_world;

        const auto v = aniso_ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v);

        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
      }

      polyhedron_ = Vs;
    }

    /// Anisotropic polyhedron (3D)
    template<int U = Dim>
    typename std::enable_if<U == 3>::type
    find_polyhedron_aniso(const Vecf<Dim> &p1, const Vecf<Dim> &p2) {
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = obs_;

        // for the first half-space, we use the original ellipsoid
      if (!obs_remain.empty()) {
        const auto v0 = ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v0);
        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v0.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
      }

      if (obs_remain.empty()) {
        polyhedron_ = Vs;
        return;
      }

      Vecf<Dim> e0 = (p2 - p1).normalized();   // long axis

      Vecf<Dim> e1;
      if (std::abs(e0(2)) < 0.9) {
        Vecf<Dim> z; z << 0, 0, 1;
        e1 = e0.cross(z).normalized();
      } else {
        Vecf<Dim> y; y << 0, 1, 0;
        e1 = e0.cross(y).normalized();
      }
      Vecf<Dim> e2 = e0.cross(e1).normalized();

      Matf<Dim, Dim> R = Matf<Dim, Dim>::Identity();
      R.col(0) = e0;  // long
      R.col(1) = e1;  // lateral-1
      R.col(2) = e2;  // lateral-2

      const double scale_long = 1.05;
      const double scale_lat  = 1.2;
      const double max_lat    = 1.0;

      while (!obs_remain.empty()) {
        Matf<Dim, Dim> C_world = aniso_ellipsoid_.C();
        Matf<Dim, Dim> C_local = R.transpose() * C_world * R;

        C_local(0,0) *= scale_long;
        C_local(1,1) *= scale_lat;
        C_local(2,2) *= scale_lat;
        C_local(1,1) = std::min(max_lat, C_local(1,1));
        C_local(2,2) = std::min(max_lat, C_local(2,2));

        for (int i = 0; i < Dim; ++i)
          for (int j = 0; j < Dim; ++j)
            if (i != j)
              C_local(i,j) = 0;

        C_world = R * C_local * R.transpose();
        aniso_ellipsoid_.C_ = C_world;

        const auto v = aniso_ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v);

        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
      }

      polyhedron_ = Vs;
    }

    /// Obstacles, input
    vec_Vecf<Dim> obs_;

    /// Output ellipsoid
    Ellipsoid<Dim> ellipsoid_;

    /// aniso ellipsoid to generate polyhedron
    Ellipsoid<Dim> aniso_ellipsoid_;

    /// Output polyhedron
    Polyhedron<Dim> polyhedron_;

    /// Local bounding box along the line segment
    Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()};

    /// Local anisotropy bounding box along the line segment
    Vecf<Dim> local_bbox_aniso_pos{Vecf<Dim>::Zero()};
    Vecf<Dim> local_bbox_aniso_neg{Vecf<Dim>::Zero()};
};
#endif
