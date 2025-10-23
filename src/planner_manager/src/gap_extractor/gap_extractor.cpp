#include "gap_extractor/gap_extractor.h"
 
static inline void fillRowSpanWrap(std::vector<std::vector<uint8_t>>& M, int v, int uL, int uR, int W)
{
    if (W <= 0) return;
    uL = (uL % W + W) % W;
    uR = (uR % W + W) % W;
    if (uR > uL) {
        for (int u = uL + 1; u <= uR; ++u) M[v][u] = 1;
    } else if (uR < uL) {
        for (int u = uL + 1; u < W; ++u) M[v][u] = 1;
        for (int u = 0; u <= uR; ++u)     M[v][u] = 1;
    }
}

static inline void fillColSpan(std::vector<std::vector<uint8_t>>& M, int u, int vU, int vD)
{
    if (vD <= vU) return;
    for (int v = vU + 1; v <= vD; ++v) M[v][u] = 1;
}

void GapExtractor::initialize(ros::NodeHandle &nh, bool env_type)
{
    node_ = nh;
    env_type_ = env_type;

    range_map_width_ = 1600; 
    range_map_height_ = 32;  
    cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // initialize range map
    range_map_.azimuth.resize(range_map_height_, std::vector<float>(range_map_width_, std::numeric_limits<float>::max()));
    range_map_.elevation.resize(range_map_height_, std::vector<float>(range_map_width_, std::numeric_limits<float>::max()));
    range_map_.range.resize(range_map_height_, std::vector<float>(range_map_width_, std::numeric_limits<float>::max()));
    // initialize gap masks
    gap_masks_.open.resize(range_map_height_, std::vector<uint8_t>(range_map_width_, 0));
    gap_masks_.limited.resize(range_map_height_, std::vector<uint8_t>(range_map_width_, 0));

    lidar_to_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf2_ros::Buffer tfBuffer_lidar;
    tf2_ros::TransformListener tfListener_lidar(tfBuffer_lidar);
    try
    {
        *(lidar_to_base_ptr_) = tfBuffer_lidar.lookupTransform("base_link", "scan", ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    const Eigen::Affine3d T_lidar_base = tf2::transformToEigen(*lidar_to_base_ptr_);
    T_lidar_base_mat_ = T_lidar_base.matrix().cast<float>();
    
    base_to_odom_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf_listener_odom_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_odom_);
    
    odom_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::odomCallback, this);
    
    velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &GapExtractor::velodyneCallback, this);

    image_pub_ = node_.advertise<sensor_msgs::Image>("/range_map_image", 1);
    edge_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/detected_edges", 1);
    mask_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/gap_masks", 1);

    gap_extractor_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::extractGapCallback, this);
    
    visualization_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::visualizationCallback, this);
}

void GapExtractor::pointCloudToRangeMap()
{
    const int H = range_map_height_;
    const int W = range_map_width_;
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    range_map_.range.assign(range_map_height_, std::vector<float>(range_map_width_, NaN));
    range_map_.azimuth.assign(range_map_height_, std::vector<float>(range_map_width_, 0.f));
    range_map_.elevation.assign(range_map_height_, std::vector<float>(range_map_width_, 0.f));

    if (!cloud_ptr_ || cloud_ptr_->empty())
    {
        ROS_WARN("[GapExtractor] Point cloud is not initialized");
        return;
    }

    // Angular bounds (sensor FoV, in the LIDAR frame)
    const float min_azimuth = -M_PI;
    const float max_azimuth =  M_PI;
    const float min_elev = -30.67f * M_PI / 180.0f;
    const float max_elev =  20.67f * M_PI / 180.0f;

    // Angular resolution and helpers
    const float azimuth_res = (max_azimuth - min_azimuth) / static_cast<float>(W);
    const float elev_res    = (max_elev    - min_elev)    / static_cast<float>(H);
    const float inv_az_res  = 1.f / azimuth_res;
    const float inv_el_res  = 1.f / elev_res;
    const float EPS = 1e-6f;

    for (int v = 0; v < range_map_height_; ++v) {
        const float phi_v = min_elev + (v + 0.5f) * elev_res;
        for (int u = 0; u < range_map_width_; ++u) {
            const float th_u = min_azimuth + (u + 0.5f) * azimuth_res;
            range_map_.azimuth[v][u]   = th_u;
            range_map_.elevation[v][u] = phi_v;
        }
    }

    // points are in scan frame, first transform to base frame then to odom frame
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    // const Eigen::Matrix4f T_base_odom_mat = T_base_odom.matrix().cast<float>();
    // pcl::transformPointCloud(*cloud_ptr_, *cloud_odom_ptr, T_base_odom_mat * T_lidar_base_mat_);

    auto write_cell = [&](int v, int u, float r, float th, float ph) {
        float &cell = range_map_.range[v][u];
        if (!std::isfinite(cell) || r < cell) {
            cell = r;
            range_map_.azimuth[v][u]   = th;
            range_map_.elevation[v][u] = ph;
        }
    };


    for (const auto& pt : cloud_ptr_->points) {
        const float x = pt.x, y = pt.y, z = pt.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        const float r = std::sqrt(x*x + y*y + z*z);
        if (!std::isfinite(r) || r <= 1e-3f) continue;

        // Spherical angles in sensor frame
        float th = std::atan2(y, x);                         // [-pi, pi]
        float ph = std::atan2(z, std::sqrt(x*x + y*y));      

        // FoV gating with safe clamp on the top edge
        if (ph < min_elev || ph > max_elev) continue;
        if (ph >= max_elev) ph = max_elev - EPS;             // avoid v == H

        // Column index with robust wrap-around
        int u = static_cast<int>(std::floor((th - min_azimuth) * inv_az_res));
        u = (u % W + W) % W;                                 // force into [0, W)

        // Row index;
        int v = static_cast<int>(std::floor((ph - min_elev) * inv_el_res));
        if (static_cast<unsigned>(v) >= static_cast<unsigned>(H)) continue;

        write_cell(v, u, r, th, ph);
    }
}

void GapExtractor::medianFilter()
{
    RangeMap filtered_map = range_map_; 
    const int H = range_map_height_;
    const int W = range_map_width_;

    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            const int uL = (u - 1 + W) % W;
            const int uR = (u + 1) % W;

            const float rC = range_map_.range[v][u];
            const float rL = range_map_.range[v][uL];
            const float rR = range_map_.range[v][uR];

            // If Center is NaN, leave it to hole-fill stage
            if (!std::isfinite(rC)) continue;

            // Need at least two finite among {L, C, R} to median
            int n = 0;
            float buf[3];
            if (std::isfinite(rL)) buf[n++] = rL;
            if (std::isfinite(rC)) buf[n++] = rC;
            if (std::isfinite(rR)) buf[n++] = rR;

            if (n < 2) continue; // not enough data to median

            float dpsi = angDist(range_map_.azimuth[v][uL], range_map_.elevation[v][uL], range_map_.azimuth[v][uR], range_map_.elevation[v][uR]);
            float rnear = std::min(std::isfinite(rL)?rL:INFINITY, std::isfinite(rR)?rR:INFINITY);
            if (!std::isfinite(rnear)){
                rnear = rC;
            }
            const float gate = edge_params_.a_h + edge_params_.b_h * rnear * std::sin(std::max(0.f, dpsi));

            if (std::isfinite(rL) && std::isfinite(rR)){
                if (std::fabs(rL - rR) > gate){
                    continue;
                }
            }
            std::nth_element(buf, buf + n/2, buf + n);
            filtered_map.range[v][u] = buf[n/2];
        }
    }
    range_map_.range.swap(filtered_map.range); 
}

void GapExtractor::fillTinyHoles()
{
    RangeMap filtered_map = range_map_; 
    const int H = range_map_height_;
    const int W = range_map_width_;

    const float abs_gate = 0.05f;           // absolute tolerance in meters
    const float rel_gate = 0.02f;           // relative tolerance vs near range (e.g., 2%)
    const float step_scale = 1.0f;          // scale for angular step contribution (can use edge_params_.b_h)

    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            if (std::isfinite(range_map_.range[v][u])) continue;

            const int uL = (u - 1 + W) % W;
            const int uR = (u + 1) % W;

            const float rL = range_map_.range[v][uL];
            const float rR = range_map_.range[v][uR];

            if (!std::isfinite(rL) || !std::isfinite(rR)) continue;

            // Angular span between L and R
            float dpsi = angDist(range_map_.azimuth[v][uL], range_map_.elevation[v][uL], range_map_.azimuth[v][uR], range_map_.elevation[v][uR]);
            float rnear = std::min(rL, rR);
            const float gate = abs_gate + rel_gate * rnear + step_scale * rnear * std::sin(std::max(0.f, dpsi));

            if (std::fabs(rL - rR) <= gate){
                filtered_map.range[v][u] = 0.5f * (rL + rR);
            }
        }
    }
    range_map_.range.swap(filtered_map.range);
}

void GapExtractor::fetchRangeForCompare(int v, int u, float r_max, float& r, bool& is_unknown)
{
    const float x = range_map_.range[v][u];
    if(std::isfinite(x)){
        r = x;
        is_unknown = false;
    } else {
        r = r_max;
        is_unknown = true;
    }
}

void GapExtractor::detectEdges()
{
    std::vector<Edge> edges;
    const int H = range_map_height_;
    const int W = range_map_width_;

    const float R_MAX = 100.0f;

    const float z_band_ground = 0.05f; 
    const float z_gate_v      = 0.05f; 
    const float curv_gate     = 0.20f; 

    auto z_of = [&](int v, int u)->float{
        const float r   = range_map_.range[v][u];
        if (!std::isfinite(r)) return std::numeric_limits<float>::quiet_NaN();
        const float phi = range_map_.elevation[v][u];
        return r * std::sin(phi);
    };

    auto try_pair = [&](int v1, int u1, int v2, int u2, int type){

        const bool horizontal = (type == 0);
        const float a = horizontal ? edge_params_.a_h : edge_params_.a_v;
        const float b = horizontal ? edge_params_.b_h : edge_params_.b_v;
        const float lambda = horizontal ? edge_params_.lambda_h : edge_params_.lambda_v;
        const float eps_diff = horizontal ? edge_params_.eps_h : edge_params_.eps_v;

        float r1c, r2c; bool unk1, unk2;
        fetchRangeForCompare(v1, u1, R_MAX, r1c, unk1);
        fetchRangeForCompare(v2, u2, R_MAX, r2c, unk2);
        if (unk1 && unk2) return; // both unknown, no edge

        HEdgeType htype = HEdgeType::NONE;
        VEdgeType vtype = VEdgeType::NONE;
        const float dr = r1c - r2c;
        if (horizontal){
            if      (dr < -eps_diff) htype = HEdgeType::L;
            else if (dr >  eps_diff) htype = HEdgeType::R;
        } else {
            if      (dr < -eps_diff) vtype = VEdgeType::U;
            else if (dr >  eps_diff) vtype = VEdgeType::D;
        }

        if ((horizontal && htype == HEdgeType::NONE) || (!horizontal && vtype == VEdgeType::NONE)) {
            return; // no significant difference
        }

        // both have finite ranges
        if (!unk1 && !unk2){
            if (!horizontal){
                // vertical edge, check z difference
                const float z1 = z_of(v1, u1);
                const float z2 = z_of(v2, u2);
                if (std::isfinite(z1) && std::isfinite(z2)){
                    if (std::fabs(z1) < z_band_ground && std::fabs(z2) < z_band_ground) return; // both on ground
                    if (std::fabs(z1 - z2) < z_gate_v) {
                        bool smooth = false;
                        // check curvature
                        if (v1 - 1 >= 0){
                            const float r_temp = range_map_.range[v1 - 1][u1];
                            if (std::isfinite(r_temp)){
                                const float curv = std::fabs(r2c - 2.f * r1c + r_temp);
                                if (curv < curv_gate) smooth = true;
                            }
                        }
                        if (!smooth && v2 + 1 < H){
                            const float r_temp = range_map_.range[v2 + 1][u2];
                            if (std::isfinite(r_temp)){
                                const float curv = std::fabs(r_temp - 2.f * r2c + r1c);
                                if (curv < curv_gate) smooth = true;
                            }
                        }
                        if (smooth) return; // smooth surface, no edge
                    }
                }
            }
            const float dpsi = angDist(range_map_.azimuth[v1][u1],   range_map_.elevation[v1][u1],
                                       range_map_.azimuth[v2][u2],   range_map_.elevation[v2][u2]);
            const float rmin = std::min(r1c, r2c);
            const float thr  = a + b * rmin * std::sin(dpsi);

            if (std::fabs(dr) <= thr) return;

            const float rnear = rmin, rfar = std::max(r1c, r2c);
            if (rfar * std::cos(dpsi) <= rnear + lambda * rnear * std::sin(dpsi)) return;

            edges.push_back(Edge{v1, u1, type, range_map_.range[v1][u1], EdgeClass::FF, htype, vtype});
        }
        else if (unk1 != unk2){
            // one finite, one unknown
            // push finite point as edge
            if (unk1) {
                edges.push_back(Edge{v2, u2, type, range_map_.range[v2][u2], EdgeClass::FU, htype, vtype});
            } else {
                edges.push_back(Edge{v1, u1, type, range_map_.range[v1][u1], EdgeClass::FU, htype, vtype});
            }
        }
    };

    // horizontal edges
    for(int v=0; v<H; ++v){
        for(int u=0; u<W; ++u){
            int u_next = (u+1)%W;
            try_pair(v, u, v, u_next, 0);
        }
    }

    // vertical edges
    for(int u=0; u<W; ++u){
        for(int v=v_margin_; v<H-1-v_margin_; ++v){
            int v_next = v+1;
            try_pair(v, u, v_next, u, 1);
        }
    }

    // size of horizontal and vertical edges
    size_t h_edge_count = 0;
    size_t v_edge_count = 0;
    for (const auto& edge : edges) {
        if (edge.type == 0) h_edge_count++;
        else if (edge.type == 1) v_edge_count++;
    }
    ROS_INFO("[GapExtractor] Detected %lu horizontal edges, %lu vertical edges", h_edge_count, v_edge_count);
    selected_edges_.clear();
    selected_edges_ = edges;
}

void GapExtractor::buildGapMasks()
{
    GapMasks gap_masks;
    const int H = range_map_height_;
    const int W = range_map_width_;
    gap_masks.open.resize(H, std::vector<uint8_t>(W, 0));
    gap_masks.limited.resize(H, std::vector<uint8_t>(W, 0));

    // horizontal scan
    std::vector<std::vector<std::tuple<int,EdgeClass,HEdgeType>>> rows(H);
    for (const auto& e : selected_edges_) {
        if (e.type != 0) continue;
        if (e.h_edge_type == HEdgeType::NONE) continue;
        rows[e.v].emplace_back(e.u, e.edge_class, e.h_edge_type);
    }

    for (int v = 0; v < H; ++v){
        auto& edge_list = rows[v];
        if (edge_list.empty()) continue;
        std::sort(edge_list.begin(), edge_list.end(),
                  [](const auto& a, const auto& b){
                      return std::get<0>(a) < std::get<0>(b);
                  });

        const int n = (int)edge_list.size();
        std::vector<uint8_t> used(n, 0);

        auto scan_and_pair = [&](EdgeClass cls, HEdgeType left_tag, HEdgeType right_tag, std::vector<std::vector<uint8_t>>& mask){
            for (int i = 0; i < n; ++i){
                if (used[i]) continue;
                const int u_i = std::get<0>(edge_list[i]);
                const EdgeClass cls_i = std::get<1>(edge_list[i]);
                const HEdgeType tag_i = std::get<2>(edge_list[i]);
                if (cls_i != cls || tag_i != left_tag) continue;

                int j = (i + 1) % n;
                bool found_pair = false;
                for (int step = 0; step < n - 1; ++step){
                    if (!used[j]){
                        const EdgeClass cls_j = std::get<1>(edge_list[j]);
                        const HEdgeType tag_j = std::get<2>(edge_list[j]);
                        if (cls_j == cls && tag_j == right_tag){
                            found_pair = true;
                            break;
                        }
                    }
                    j = (j + 1) % n;
                }
                if(!found_pair) continue;

                const int u_j = std::get<0>(edge_list[j]);
                fillRowSpanWrap(mask, v, u_i, u_j, W);
                used[i] = 1;
                used[j] = 1;
            }
        };
        // open gaps: FF-L and FF-R
        scan_and_pair(EdgeClass::FU, HEdgeType::L, HEdgeType::R, gap_masks.open);
        // limited gaps: FU-L and FU-R
        // scan_and_pair(EdgeClass::FF, HEdgeType::L, HEdgeType::R, gap_masks.limited);
    }

    // vertical scan
    std::vector<std::vector<std::tuple<int,EdgeClass,VEdgeType>>> cols(W);
    for (const auto& e : selected_edges_) {
        if (e.type != 1) continue;
        if (e.v_edge_type == VEdgeType::NONE) continue;
        if (e.v < v_margin_ || e.v >= H - v_margin_) continue;
        cols[e.u].emplace_back(e.v, e.edge_class, e.v_edge_type);
    }

    for (int u = 0; u < W; ++u){
        auto& C = cols[u];
        if (C.empty()) continue;
        std::sort(C.begin(), C.end(),
                  [](const auto& a, const auto& b){
                      return std::get<0>(a) < std::get<0>(b);
                  });
        const int n = (int)C.size();
        std::vector<uint8_t> used(n, 0);

        auto scan_and_pair_col = [&](EdgeClass cls, VEdgeType up_tag, VEdgeType down_tag, std::vector<std::vector<uint8_t>>& mask){
            for (int i = 0; i < n; ++i){
                if (used[i]) continue;
                const int v_i = std::get<0>(C[i]);
                const EdgeClass cls_i = std::get<1>(C[i]);
                const VEdgeType tag_i = std::get<2>(C[i]);
                if (cls_i != cls || tag_i != up_tag) continue;

                int j = i + 1;
                while (j < n){
                    if (!used[j]){
                        const EdgeClass cls_j = std::get<1>(C[j]);
                        const VEdgeType tag_j = std::get<2>(C[j]);
                        if (cls_j == cls && tag_j == down_tag){
                            break;
                        }
                    }
                    ++j;
                }
                if (j >= n) return;
                const int v_j = std::get<0>(C[j]);
                fillColSpan(mask, u, v_i, v_j);
                used[i] = 1;
                used[j] = 1;
            }
        };
        // open gaps: FF-U and FF-D
        scan_and_pair_col(EdgeClass::FU, VEdgeType::U, VEdgeType::D, gap_masks.open);
        // limited gaps: FU-U and FU-D
        // scan_and_pair_col(EdgeClass::FF, VEdgeType::U, VEdgeType::D, gap_masks.limited);
    }

    buildGapMasks_FromSingleFFEdge(gap_masks.limited);

    gap_masks_.open.clear();
    gap_masks_.limited.clear();
    gap_masks_.open = gap_masks.open;
    gap_masks_.limited = gap_masks.limited;
}

void GapExtractor::buildGapMasks_FromSingleFFEdge(std::vector<std::vector<uint8_t>>& mask_limited){
    const int H = range_map_height_;
    const int W = range_map_width_;
    const float a = 0.08f;
    const float b = 0.02f;
    const float lambda = 0.5f;
    const float eps_diff = 0.03f;

    auto sweep_row = [&](int v, int u_start, int step, int u_near, float r_near){
        std::vector<int> path;
        path.reserve(12);
        int u = u_start;
        float r_prev = r_near;
        const float azN = range_map_.azimuth[v][u_near];
        const float phN = range_map_.elevation[v][u_near];

        for (int k = 0; k < 12; ++k){
            if (u < 0)
                u += W;
            else if (u >= W)
                u -= W;
            const float r = range_map_.range[v][u];
            if (!std::isfinite(r)) break;

            const float dpsi = angDist(azN, phN, range_map_.azimuth[v][u], range_map_.elevation[v][u]);
            const float threshold = a + b * r_near * std::sin(dpsi);
            const bool pass_geom = (r >= r_near + threshold);
            const bool pass_mono = (r >= r_prev - eps_diff);

            if (!pass_geom || !pass_mono) break;
            path.push_back(u);
            r_prev = r;
            u += step;
        }
        if (path.size() >= 3){
            int mid = path[(int)path.size()/2];
            mask_limited[v][mid] = 1;
        }
    };

    auto sweep_col = [&](int u, int v_start, int step, int v_near, float r_near){
        std::vector<int> path;
        path.reserve(12);

        int v = v_start;
        float r_prev = r_near;
        const float azN = range_map_.azimuth[v_near][u];
        const float phN = range_map_.elevation[v_near][u];

        for (int k = 0; k < 12; ++k){
            if (v < v_margin_ || v >= H - v_margin_) break;
            const float r = range_map_.range[v][u];
            if (!std::isfinite(r)) break;

            const float dpsi = angDist(azN, phN, range_map_.azimuth[v][u], range_map_.elevation[v][u]);
            const float threshold = a + b * r_near * std::sin(dpsi);
            const bool pass_geom = (r >= r_near + threshold);
            const bool pass_mono = (r >= r_prev - eps_diff);
            if (!pass_geom || !pass_mono) break;
            path.push_back(v);
            r_prev = r;
            v += step;
        }
        if (path.size() >= 3){
            int mid = path[(int)path.size()/2];
            mask_limited[mid][u] = 1;
        }
    };

    for (const auto& e : selected_edges_){
        if (e.edge_class != EdgeClass::FF) continue;
        const int v = e.v, u = e.u;

        if (e.type == 0){
            // horizontal edge
            if (e.h_edge_type == HEdgeType::NONE) continue;
            const int uR = (u + 1) % W;
            float rL = range_map_.range[v][u];
            float rR = range_map_.range[v][uR];

            if (e.h_edge_type == HEdgeType::L){
                const float r_near = std::isfinite(rL) ? rL : rR;
                sweep_row(v, uR, +1, u, r_near);
            }
            else{
                const float r_near = std::isfinite(rR) ? rR : rL;
                sweep_row(v, u, -1, uR, r_near);
            }
        }
        else if (e.type == 1){
            // vertical edge
            if (e.v_edge_type == VEdgeType::NONE) continue;
            const int vD = v + 1;
            if (vD >= H) continue;
            float rU = range_map_.range[v][u];
            float rD = range_map_.range[vD][u];

            if (e.v_edge_type == VEdgeType::U){
                const float r_near = std::isfinite(rU) ? rU : rD;
                sweep_col(u, vD, +1, v, r_near);
            }
            else{
                const float r_near = std::isfinite(rD) ? rD : rU;
                sweep_col(u, v, -1, vD, r_near);
            }
        }
    }
}

void GapExtractor::extractGapCallback(const ros::TimerEvent &e){
    // project 3D point cloud to 2D range map
    pointCloudToRangeMap();
    if (range_map_.range.empty()){
        ROS_WARN("[GapExtractor] Range map is empty");
        return;
    }
    // preprocess range map
    medianFilter();
    fillTinyHoles();
    // detect edges
    detectEdges();
    // build gap masks
    buildGapMasks();
}

inline float GapExtractor::angDist(float th1, float ph1, float th2, float ph2)
{
    const float s1 = std::sin(ph1), c1 = std::cos(ph1);
    const float s2 = std::sin(ph2), c2 = std::cos(ph2);
    const float dth = th1 - th2;
    float c = s1*s2 + c1*c2*std::cos(dth);
    c = std::max(-1.0f, std::min(1.0f, c));
    return std::acos(c);
}

void GapExtractor::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // Process the Velodyne point cloud data
    if (msg->data.size() == 0)
    {
        ROS_WARN("[GapExtractor] Received empty point cloud data");
        return;
    }
    // process point cloud to pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::fromROSMsg(*msg, *cloud_ptr_);
}

void GapExtractor::odomCallback(const ros::TimerEvent &event)
{
    try{
        *base_to_odom_ptr_ = tf_buffer_odom_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void GapExtractor::publishRangeMapAsImage()
{
    if (range_map_.range.empty()){
        ROS_WARN("[GapExtractor] Range map is empty");
        return;
    }
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = "scan";
    img_msg.height = range_map_height_;
    img_msg.width = range_map_width_;
    img_msg.encoding = "mono8";
    img_msg.is_bigendian = false;
    img_msg.step = range_map_width_;
    img_msg.data.resize(range_map_height_ * range_map_width_);

    float min_val = std::numeric_limits<float>::max();
    float max_val = 0;
    for (int v = 0; v < range_map_height_; ++v)
        for (int u = 0; u < range_map_width_; ++u)
            if (range_map_.range[v][u] < std::numeric_limits<float>::max()) {
                min_val = std::min(min_val, range_map_.range[v][u]);
                max_val = std::max(max_val, range_map_.range[v][u]);
            }
    if (min_val >= max_val) min_val = 0, max_val = min_val + 1.0f;

    for (int v = 0; v < range_map_height_; ++v)
    {
        for (int u = 0; u < range_map_width_; ++u)
        {
            float val = range_map_.range[v][u];
            uint8_t pixel = 0;
            if (val < std::numeric_limits<float>::max())
                pixel = static_cast<uint8_t>(255.0f * (val - min_val) / (max_val - min_val));
            img_msg.data[v * range_map_width_ + u] = pixel;
        }
    }
    image_pub_.publish(img_msg);
}

void GapExtractor::publishEdges(){
    // publish edges as a single marker (points) to avoid duplicate/accumulating markers
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    marker.header.frame_id = "scan";
    marker.header.stamp = ros::Time::now();
    marker.ns = "detected_edges";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.points.clear();
    for (const auto& edge : selected_edges_){
        // only visualize vertical edges
        // if (edge.type != 1) continue;
        // only visualize horizontal edges
        if (edge.type != 0) continue;

        float r = edge.r;
        // skip edges with invalid range (could be NaN for unknown)
        if (!std::isfinite(r)) continue;

        geometry_msgs::Point p;
        const float th = range_map_.azimuth[edge.v][edge.u];
        const float ph = range_map_.elevation[edge.v][edge.u];
        p.x = r * std::cos(ph) * std::cos(th);
        p.y = r * std::cos(ph) * std::sin(th);
        p.z = r * std::sin(ph);
        marker.points.push_back(p);
    }

    // push the single marker containing all points
    marker_array.markers.push_back(marker);
    ROS_INFO("[GapExtractor] Publishing %lu edges", marker.points.size());
    edge_pub_.publish(marker_array);
}

visualization_msgs::Marker GapExtractor::maskToMarkerPoints(const std::vector<std::vector<uint8_t>>& mask, 
                                const RangeMap& range_map, float radius,
                            float r, float g, float b, int id)
{
    visualization_msgs::Marker mask_marker;
    mask_marker.header.frame_id = "scan";
    mask_marker.ns = "gap_masks";
    mask_marker.id = id;
    mask_marker.type = visualization_msgs::Marker::POINTS;
    mask_marker.scale.x = 0.01;
    mask_marker.scale.y = 0.01;
    mask_marker.color.r = r;
    mask_marker.color.g = g;
    mask_marker.color.b = b;
    mask_marker.color.a = 1.0;

    const int H = mask.size();
    const int W = mask[0].size();
    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            if (!mask[v][u]) continue;
            float th = range_map.azimuth[v][u];
            float ph = range_map.elevation[v][u];
            geometry_msgs::Point p;
            p.x = radius * std::cos(ph) * std::cos(th);
            p.y = radius * std::cos(ph) * std::sin(th);
            p.z = radius * std::sin(ph);
            mask_marker.points.push_back(p);
        }
    }
    return mask_marker;
}

void GapExtractor::publishMasks()
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(
        maskToMarkerPoints(gap_masks_.open, range_map_, 3.0f, 0.0f, 1.0f, 0.0f, 0)
    );
    marker_array.markers.push_back(
        maskToMarkerPoints(gap_masks_.limited, range_map_, 1.5f, 0.0f, 0.5f, 0.5f, 1)
    );
    mask_pub_.publish(marker_array);
}

void GapExtractor::visualizationCallback(const ros::TimerEvent &e)
{
    publishRangeMapAsImage();
    publishEdges();
    publishMasks();
}