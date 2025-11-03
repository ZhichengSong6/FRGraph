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

float circularSpanRad(const std::vector<float>& angles) {
    const size_t n = angles.size();
    if (n < 2) return 0.f;

    const float TWO_PI = 2.f * static_cast<float>(M_PI);

    // 1) Normalize all angles into [0, 2π)
    std::vector<float> a;
    a.reserve(n);
    for (float t : angles) {
        float w = std::fmod(t, TWO_PI);  // w in (-2π, 2π)
        if (w < 0.f) w += TWO_PI;        // now in [0, 2π)
        a.push_back(w);
    }

    // 2) Sort on the circle
    std::sort(a.begin(), a.end());

    // 3) Find the largest gap between consecutive angles (including wrap-around)
    float max_gap = 0.f;
    for (size_t i = 1; i < a.size(); ++i) {
        float gap = a[i] - a[i - 1];     // float
        if (gap > max_gap) max_gap = gap;
    }
    // Wrap-around gap between last and first (through 2π)
    float wrap_gap = (a.front() + TWO_PI) - a.back();
    if (wrap_gap > max_gap) max_gap = wrap_gap;

    // 4) The minimal arc covering all angles = 2π - largest gap
    float span = TWO_PI - max_gap;
    if (span < 0.f)      span = 0.f;
    else if (span > TWO_PI) span = TWO_PI;

    return span;
}

// Linear span (max - min) for elevation
static float linearSpanRad(const std::vector<float> &vals) {
    if (vals.empty()) return 0.f;
    float vmin = vals[0], vmax = vals[0];
    for (float v : vals) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }
    return (vmax - vmin);
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
    range_map_.z_world.resize(range_map_height_, std::vector<float>(range_map_width_, std::numeric_limits<float>::max()));
    // initialize gap masks
    gap_masks_.open.resize(range_map_height_, std::vector<uint8_t>(range_map_width_, 0));
    gap_masks_.limited.resize(range_map_height_, std::vector<uint8_t>(range_map_width_, 0));
    gap_regions_.clear();

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

    range_map_.range.assign(H, std::vector<float>(W, NaN));
    range_map_.azimuth.assign(H, std::vector<float>(W, 0.f));
    range_map_.elevation.assign(H, std::vector<float>(W, 0.f));
    range_map_.z_world.assign(H, std::vector<float>(W, NaN));

    if (!cloud_ptr_ || cloud_ptr_->empty())
    {
        ROS_WARN("[GapExtractor] Point cloud is not initialized");
        return;
    }

    // Field of view (radians)
    const float min_azimuth = -M_PI;
    const float max_azimuth =  M_PI;
    const float min_elev = -30.67f * M_PI / 180.0f;
    const float max_elev =  20.67f * M_PI / 180.0f;

    // Angular resolutions
    const float azimuth_res = (max_azimuth - min_azimuth) / static_cast<float>(W);
    const float elev_res    = (max_elev    - min_elev)    / static_cast<float>(H);
    const float inv_az_res  = 1.f / azimuth_res;
    const float inv_el_res  = 1.f / elev_res;
    const float EPS = 1e-6f;                 // tiny epsilon for clamping
    const float two_pi = 2.f * M_PI;

    // Initialize pixel-center angles (for reference/visualization)
    for (int v = 0; v < H; ++v) {
        const float phi_v = min_elev + (v + 0.5f) * elev_res;
        for (int u = 0; u < W; ++u) {
            const float th_u = min_azimuth + (u + 0.5f) * azimuth_res;
            range_map_.azimuth[v][u]   = th_u;
            range_map_.elevation[v][u] = phi_v;
        }
    }

    // Build LiDAR->odom transform once (points are in LiDAR frame)
    Eigen::Matrix4f T_lidar_odom = Eigen::Matrix4f::Identity();
    {
        const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
        const Eigen::Matrix4f T_base_odom_mat = T_base_odom.matrix().cast<float>();
        T_lidar_odom = T_base_odom_mat * T_lidar_base_mat_; // LiDAR -> base -> odom
    }

    auto write_cell = [&](int v, int u, float r, float th, float ph, float z_world) {
        float &cell = range_map_.range[v][u];
        if (!std::isfinite(cell) || r < cell) {
            cell = r;
            range_map_.azimuth[v][u]   = th;
            range_map_.elevation[v][u] = ph;
            range_map_.z_world[v][u] = z_world;
        }
    };

    float ph_min_obs =  1e9f, ph_max_obs = -1e9f;
    for (const auto& pt : cloud_ptr_->points) {
        const float x = pt.x, y = pt.y, z = pt.z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        const float r = std::sqrt(x*x + y*y + z*z);
        if (!std::isfinite(r) || r <= 1e-3f) continue;

        // Spherical coordinates in the sensor frame
        float th = std::atan2(y, x);                       // [-pi, pi]
        float ph = std::atan2(z, std::sqrt(x*x + y*y));    // [-pi/2, pi/2]
        ph_min_obs = std::min(ph_min_obs, ph);
        ph_max_obs = std::max(ph_max_obs, ph);

        // ---- Clamp to (min, max) BEFORE binning (avoid half-open interval leak) ----
        // Elevation: clamp to (min_elev, max_elev)
        float ph_c = std::min(std::max(ph, min_elev + EPS), max_elev - EPS);

        // Azimuth: wrap into [min,max), then clamp slightly inside, then round-to-center
        float th_w = th;
        while (th_w <  min_azimuth) th_w += two_pi;
        while (th_w >= max_azimuth) th_w -= two_pi;
        th_w = std::min(std::max(th_w, min_azimuth + EPS), max_azimuth - EPS);

        // ---- Row/column indices: round to pixel centers (round-to-center) ----
        // Row centers at min_elev + (v+0.5)*elev_res
        const float v_float = (ph_c - (min_elev + 0.5f * elev_res)) * inv_el_res;
        int v = static_cast<int>(std::floor(v_float + 0.5f));   // round
        if (v < 0) v = 0; else if (v >= H) v = H - 1;

        // Column centers at min_azimuth + (u+0.5)*azimuth_res
        const float u_float = (th_w - (min_azimuth + 0.5f * azimuth_res)) * inv_az_res;
        int u = static_cast<int>(std::floor(u_float + 0.5f));   // round
        // Wrap around horizontally
        u = (u % W + W) % W;

        // odom z
        Eigen::Vector4f p_lidar(x, y, z, 1.f);
        const float z_world = (T_lidar_odom * p_lidar).z();

        // Store original (unclamped) angles for downstream geometry if needed
        write_cell(v, u, r, th, ph, z_world);
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
                filtered_map.range[v][u] = std::min(rL, rR);
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

    const float R_MAX = 10.0f;

    static constexpr float z_curv_gate = 30.0f;

    auto kappaZTriplet = [&](int vc, int uc)->float {
        if (vc - 1 < 0 || vc + 1 >= H) return std::numeric_limits<float>::infinity();

        const float r_up  = range_map_.range[vc - 1][uc];
        const float r_mid = range_map_.range[vc][uc];
        const float r_dn  = range_map_.range[vc + 1][uc];

        const float ph_up  = range_map_.elevation[vc - 1][uc];
        const float ph_mid = range_map_.elevation[vc][uc];
        const float ph_dn  = range_map_.elevation[vc + 1][uc];

        if (!std::isfinite(r_up)  || !std::isfinite(r_mid) || !std::isfinite(r_dn) ||
            !std::isfinite(ph_up) || !std::isfinite(ph_mid)|| !std::isfinite(ph_dn))
            return std::numeric_limits<float>::infinity();

        const float z_up  = r_up  * std::sin(ph_up);
        const float z_mid = r_mid * std::sin(ph_mid);
        const float z_dn  = r_dn  * std::sin(ph_dn);

        const float dphi1 = std::fabs(ph_mid - ph_up);
        const float dphi2 = std::fabs(ph_dn  - ph_mid);
        const float dphi  = 0.5f * (dphi1 + dphi2);
        const float denom = std::max(1e-3f, dphi * dphi);   // 角步长归一

        return std::fabs(z_up - 2.0f * z_mid + z_dn) / denom;
    };

    auto try_pair = [&](int v1, int u1, int v2, int u2, int type){

        const bool horizontal = (type == 0);
        const float a = horizontal ? edge_params_.a_h : edge_params_.a_v;
        const float b = horizontal ? edge_params_.b_h : edge_params_.b_v;
        const float lambda = horizontal ? edge_params_.lambda_h : edge_params_.lambda_v;
        // const float eps_diff = horizontal ? edge_params_.eps_h : edge_params_.eps_v;

        float r1c, r2c; bool unk1, unk2;
        fetchRangeForCompare(v1, u1, R_MAX, r1c, unk1);
        fetchRangeForCompare(v2, u2, R_MAX, r2c, unk2);

        if (unk1 && unk2) return; // both unknown, no edge

        const float dr = r1c - r2c;

        if (unk1 ^ unk2){
            // one finite, one unknown
            const int vv = unk1 ? v2 : v1;
            const int uu = unk1 ? u2 : u1;
            const bool dr_pos = (dr > 0.f);

            HEdgeType htype = HEdgeType::NONE;
            VEdgeType vtype = VEdgeType::NONE;
            if (horizontal) { htype = dr_pos ? HEdgeType::R : HEdgeType::L; }
            else            { vtype = dr_pos ? VEdgeType::D : VEdgeType::U; }

            edges.push_back(Edge{vv, uu, type, range_map_.range[vv][uu], EdgeClass::FU, htype, vtype});
            return;
        }

        // both finite
        const float th1 = range_map_.azimuth  [v1][u1];
        const float ph1 = range_map_.elevation[v1][u1];
        const float th2 = range_map_.azimuth  [v2][u2];
        const float ph2 = range_map_.elevation[v2][u2];
        if (!std::isfinite(th1) || !std::isfinite(ph1) || !std::isfinite(th2) || !std::isfinite(ph2)) return;

        const float dpsi  = angDist(th1, ph1, th2, ph2);
        const float rnear = std::min(r1c, r2c);
        const float rfar  = std::max(r1c, r2c);

        const float thr  = a + b * rnear * std::sin(std::max(0.f, dpsi));
        if (std::fabs(dr) <= thr) return;
        if (rfar * std::cos(dpsi) <= rnear + lambda * rnear * std::sin(std::max(0.f, dpsi))) return;

        if (!horizontal) {

            const int vm_near = (r1c <= r2c) ? v1 : v2;
            const int um_near = (r1c <= r2c) ? u1 : u2; 

            const float k1 = kappaZTriplet(v1,     um_near);
            const float k2 = kappaZTriplet(v2,     um_near);
            const float k3 = kappaZTriplet(vm_near,um_near);
            const float kappa_z = std::min(k1, std::min(k2, k3));

            if (kappa_z < z_curv_gate) return;  // same surface
        }

        HEdgeType htype = HEdgeType::NONE;
        VEdgeType vtype = VEdgeType::NONE;
        if (horizontal) { htype = (dr > 0.f) ? HEdgeType::R : HEdgeType::L; }
        else            { vtype = (dr > 0.f) ? VEdgeType::D : VEdgeType::U; }

        const int va = (r1c <= r2c) ? v1 : v2;
        const int ua = (r1c <= r2c) ? u1 : u2;
        edges.push_back(Edge{va, ua, type, range_map_.range[va][ua], EdgeClass::FF, htype, vtype});
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
    // size_t h_edge_count = 0;
    // size_t v_edge_count = 0;
    // for (const auto& edge : edges) {
    //     if (edge.type == 0) h_edge_count++;
    //     else if (edge.type == 1) v_edge_count++;
    // }
    // ROS_INFO("[GapExtractor] Detected %lu horizontal edges, %lu vertical edges", h_edge_count, v_edge_count);
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

        // helper function: find a "safe" start, to deal with with wrap-around(LRRL)
        auto find_safe_start = [&](EdgeClass cls)->int{
            for (int i = 0; i < n; ++i){
                int prev = (i - 1 + n) % n;
                const auto& ep = edge_list[prev];
                const auto& ei = edge_list[i];
                if (std::get<1>(ep) == cls && std::get<2>(ep) == HEdgeType::R &&
                    std::get<1>(ei) == cls && std::get<2>(ei) == HEdgeType::L){
                    return i; // 
                }
            }
            return 0;
        };

        // ---- helper: stack-based pairing on a circular row (no crossing) ----
        auto pair_row_stack = [&](EdgeClass cls, std::vector<std::vector<uint8_t>>& mask){
            if (edge_list.empty()) return;
            const int s = find_safe_start(cls);

            auto idx = [&](int k){ return (s + k) % n; };

            std::vector<int> Lstack;
            Lstack.reserve(n);

            for (int kk = 0; kk < n; ++kk) {
                const int i = idx(kk);
                if (used[i]) continue;

                const auto& ei = edge_list[i];
                const int        u_i  = std::get<0>(ei);
                const EdgeClass  c_i  = std::get<1>(ei);
                const HEdgeType  t_i  = std::get<2>(ei);
                if (c_i != cls) continue;

                if (t_i == HEdgeType::L) {
                    // push this L (unmatched so far)
                    Lstack.push_back(i);
                } else if (t_i == HEdgeType::R) {
                    // pop the nearest unmatched L
                    while (!Lstack.empty() && used[Lstack.back()]) Lstack.pop_back();
                    if (Lstack.empty()) {
                        // stray R; ignore
                        continue;
                    }
                    const int il = Lstack.back();
                    Lstack.pop_back();

                    if (used[il]) continue; // double check

                    const int u_l = std::get<0>(edge_list[il]);
                    const int u_r = u_i;

                    // fill span on a circular row (wrap-aware)
                    fillRowSpanWrap(mask, v, u_l, u_r, W);

                    used[il] = 1;
                    used[i]  = 1;
                }
            }
        };

        // ---- OPEN gaps use FU with L→R only (your current semantics) ----
        pair_row_stack(EdgeClass::FU, gap_masks.open);
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
                if (j >= n) continue;
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

    // Ensure mask size and clear to zeros
    if ((int)mask_limited.size() != H) {
        mask_limited.assign(H, std::vector<uint8_t>(W, 0));
    } else {
        for (int v = 0; v < H; ++v) {
            std::fill(mask_limited[v].begin(), mask_limited[v].end(), 0);
        }
    }

    // Set seeds at near-side pixels of all FF edges
    for (const auto& e : selected_edges_) {
        if (e.edge_class != EdgeClass::FF) continue;

        int v = e.v;
        int u = e.u;

        // Bounds/wrap checks
        if (v < 0 || v >= H) continue;
        u %= W; if (u < 0) u += W;

        mask_limited[v][u] = 1;
    }
}

void GapExtractor::extractGapRegions(int min_pixels)
{
    gap_regions_.clear();
    const int H = range_map_height_;
    const int W = range_map_width_;

    // first only process open gaps
    if (gap_masks_.open.empty() || (int)gap_masks_.open.size() != H) return;

    // get a visited map
    std::vector<std::vector<uint8_t>> visited(H, std::vector<uint8_t>(W, 0));

    // scan all cells
    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            if (!gap_masks_.open[v][u]) continue; // not a gap cell
            if (visited[v][u]) continue;          // already visited

            GapRegion region;
            // BFS queue
            bfsOpenGapRegion(v, u, visited, region);

            if (region.size >= min_pixels){
                gap_regions_.emplace_back(std::move(region));
            }
        }
    }
    ROS_INFO("[GapExtractor] Extracted %lu open gap regions", gap_regions_.size());
}

void GapExtractor::bfsOpenGapRegion(int v0, int u0, std::vector<std::vector<uint8_t>>& visited, GapRegion& region){
    const int H = range_map_height_;
    const int W = range_map_width_;

    std::queue<std::pair<int,int>> q;

    visited[v0][u0] = 1;
    q.emplace(v0, u0);

    // Accumulators for spherical mean and angle spans
    float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
    std::vector<float> az_list;
    std::vector<float> el_list;
    az_list.reserve(128);
    el_list.reserve(128);

    while (!q.empty()){
        std::pair<int,int> p = q.front();
        int v = p.first;
        int u = p.second;
        q.pop();

        region.pixels.emplace_back(v, u);
        region.size++;
        region.v_min = std::min(region.v_min, v);
        region.v_max = std::max(region.v_max, v);

        const float yaw = range_map_.azimuth[v][u];
        const float elev = range_map_.elevation[v][u];

        float x, y, z;
        x = std::cos(elev) * std::cos(yaw);
        y = std::cos(elev) * std::sin(yaw);
        z = std::sin(elev);
        sum_x += x;
        sum_y += y;
        sum_z += z;
        az_list.push_back(yaw);
        el_list.push_back(elev);

        // visit neighbors (8-connectivity)
        for (int dv = -1; dv <= 1; ++dv){
            for (int du = -1; du <= 1; ++du){
                if (dv == 0 && du == 0) continue;
                int vv = v + dv;
                if (vv < 0 || vv >= H) continue;
                int uu = (u + du + W) % W; // wrap around
                if (!gap_masks_.open[vv][uu]) continue; // not a gap cell
                if (visited[vv][uu]) continue;          // already visited
                visited[vv][uu] = 1;
                q.emplace(vv, uu);
            }
        }
    }

    // Spherical mean direction
    const float norm = std::sqrt(sum_x*sum_x + sum_y*sum_y + sum_z*sum_z);
    if (norm > 1e-6f){
        region.dir_x = sum_x / norm;
        region.dir_y = sum_y / norm;
        region.dir_z = sum_z / norm;
        region.center_yaw = std::atan2(region.dir_y, region.dir_x);
        region.center_elev = std::atan2(region.dir_z, std::sqrt(region.dir_x*region.dir_x + region.dir_y*region.dir_y));
    }else{
        region.dir_x = 0.f;
        region.dir_y = 0.f;
        region.dir_z = 1.f;
        region.center_yaw = 0.f;
        region.center_elev = 0.f;
    }

    // Angular spans
    region.yaw_span = circularSpanRad(az_list);
    region.elev_span = linearSpanRad(el_list);
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
    // from gap masks, extract gap regions
    extractGapRegions(6);
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
    marker.points.clear();
    marker.colors.clear();

    for (const auto& edge : selected_edges_){
        // compute 3D point in scan frame
        float r = edge.r;
        // skip edges with invalid range (could be NaN for unknown)
        if (!std::isfinite(r)) continue;

        geometry_msgs::Point p;
        const float th = range_map_.azimuth[edge.v][edge.u];
        const float ph = range_map_.elevation[edge.v][edge.u];
        p.x = r * std::cos(ph) * std::cos(th);
        p.y = r * std::cos(ph) * std::sin(th);
        p.z = r * std::sin(ph);

        // choose color for four categories:
        // horizontal FF -> red
        // horizontal FU -> green
        // vertical   FF -> blue
        // vertical   FU -> yellow
        std_msgs::ColorRGBA col;
        col.a = 1.0f;
        if (edge.type == 0) { // horizontal
            if (edge.edge_class == EdgeClass::FF) {
                col.r = 1.0f; col.g = 0.0f; col.b = 0.0f;
            } else { // FU
                col.r = 0.0f; col.g = 1.0f; col.b = 0.0f;
            }
        } else if (edge.type == 1) { // vertical
            if (edge.edge_class == EdgeClass::FF) {
                col.r = 0.0f; col.g = 0.0f; col.b = 1.0f;
            } else { // FU
                col.r = 1.0f; col.g = 1.0f; col.b = 0.0f;
            }
        } else {
            // fallback color (white)
            col.r = col.g = col.b = 1.0f;
        }

        marker.points.push_back(p);
        marker.colors.push_back(col);
    }
    // push the single marker containing all points
    marker_array.markers.push_back(marker);
    // ROS_INFO("[GapExtractor] Publishing %lu edges", marker.points.size());
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
        maskToMarkerPoints(gap_masks_.limited, range_map_, 0.5f, 0.0f, 0.5f, 0.5f, 1)
    );
    mask_pub_.publish(marker_array);
}

void GapExtractor::visualizationCallback(const ros::TimerEvent &e)
{
    publishRangeMapAsImage();
    publishEdges();
    publishMasks();
}