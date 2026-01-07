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

static float linearSpanRad(const std::vector<float> &vals) {
    if (vals.empty()) return 0.f;
    float vmin = vals[0], vmax = vals[0];
    for (float v : vals) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }
    return (vmax - vmin);
}

static inline float normalizeTo2Pi(float a) {
    const float TWO_PI = 2.f * static_cast<float>(M_PI);
    float w = std::fmod(a, TWO_PI);
    if (w < 0.f) w += TWO_PI;
    return w;
}

static inline float yawUnwrapOffset(const std::vector<float>& yaws){
    const size_t n = yaws.size();
    if (n < 2) return 0.f;

    const float TWO_PI = 2.f * static_cast<float>(M_PI);

    // Copy + normalize + sort
    std::vector<float> a;
    a.reserve(n);
    for (size_t i = 0; i < n; ++i){
        a.push_back(normalizeTo2Pi(yaws[i]));
    }
    std::sort(a.begin(), a.end());

    // Find largest gap
    float max_gap = 0.f;
    size_t max_gap_idx = 0;
    for (size_t i = 1; i < n; ++i){
        float g = a[i] - a[i-1];
        if (g > max_gap){
            max_gap = g;
            max_gap_idx = i;
        }
    }
    float wrap_gap = (a.front() + TWO_PI) - a.back();
    if (wrap_gap > max_gap){
        max_gap = wrap_gap;
        max_gap_idx = 0;
    }
    // Place the cut at the middle of the largest gap: offset = -cut_angle
    float cut_angle;
    if (max_gap_idx == 0) {
        // gap between a.back() and a.front()
        cut_angle = normalizeTo2Pi(0.5f * (a.back() + (a.front() + TWO_PI)));
    } else {
        cut_angle = 0.5f * (a[max_gap_idx - 1] + a[max_gap_idx]);
    }
    // We want to shift all angles by (-cut_angle), so that [cut_angle, cut_angle+2π) maps to [0,2π)
    return cut_angle;
}

static inline float yawShiftedByOffset(float yaw, float offset) {
    // shift yaw by -offset, wrapped to [0, 2π)
    float t = std::fmod(yaw - offset, 2.f * M_PI);
    if (t < 0.f) t += 2.f * M_PI;
    return t;
}

static inline void quaternionToRPY(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

void GapExtractor::initialize(ros::NodeHandle &nh, bool env_type)
{
    node_ = nh;
    env_type_ = env_type;

    node_.param<float>("lidar/min_elev_angle",                                              params_.min_elev, -30.67f);    // degrees
    node_.param<float>("lidar/max_elev_angle",                                              params_.max_elev, 20.67f);     // degrees
    node_.param<float>("gap_extractor/limited_gap_bias_yaw",                                params_.limited_gap_bias_yaw, M_PI / 36); // 5 degrees
    node_.param<float>("gap_extractor/limited_gap_bias_elev",                               params_.limited_gap_bias_elev, M_PI / 36); // 5 degrees

    if(env_type){
        node_.param<int>  ("gap_extractor/3D/min_pixels_in_open_gap_region",                params_.min_pixels_in_open_gap_region, 30);
        node_.param<float>("gap_extractor/3D/open_gap_yaw_span",                            params_.open_gap_yaw_span, M_PI / 4); // 45 degrees
        node_.param<float>("gap_extractor/3D/open_gap_elev_span",                           params_.open_gap_elev_span, M_PI / 9); // 20 degrees

        node_.param<int>  ("gap_extractor/3D/min_pixels_in_subregion",                      params_.min_pixels_in_subregion, 20);
        node_.param<int>  ("gap_extractor/3D/range_map_width",                              params_.range_map_width, 1600);  // lidar horizontal resolution
        node_.param<int>  ("gap_extractor/3D/range_map_height",                             params_.range_map_height, 32);  // lidar vertical resolution
        node_.param<int>  ("gap_extractor/3D/map_size",                                     params_.map_size, 5); // meters

        node_.param<float>("gap_extractor/3D/yaw_split_threshold_in_limited_gap_region",    params_.yaw_split_threshold_in_limited_gap_region, M_PI / 6); // 30 degrees
        node_.param<float>("gap_extractor/3D/elev_split_threshold_in_limited_gap_region",   params_.elev_split_threshold_in_limited_gap_region, M_PI / 9); // 20 degrees
        node_.param<int>  ("gap_extractor/3D/min_pixels_in_limited_gap_region",             params_.min_pixels_in_limited_gap_region, 48);
        node_.param<float>("gap_extractor/3D/limited_gap_yaw_span",                         params_.limited_gap_yaw_span, M_PI / 6); // 30 degrees
        node_.param<float>("gap_extractor/3D/limited_gap_elev_span",                        params_.limited_gap_elev_span, M_PI / 6); // 30 degrees
        node_.param<int>  ("gap_extractor/3D/min_pixels_in_limited_subregion",              params_.min_pixels_in_limited_subregion, 32);
    }
    else{
        node_.param<int>  ("gap_extractor/2D/min_pixels_in_open_gap_region",                params_.min_pixels_in_open_gap_region, 15);
        node_.param<float>("gap_extractor/2D/open_gap_yaw_span",                            params_.open_gap_yaw_span, M_PI / 4); // 45 degrees
        node_.param<float>("gap_extractor/2D/open_gap_elev_span",                           params_.open_gap_elev_span, M_PI / 9); // 20 degrees

        node_.param<int>  ("gap_extractor/2D/min_pixels_in_subregion",                      params_.min_pixels_in_subregion, 20);
        node_.param<int>  ("gap_extractor/2D/range_map_width",                              params_.range_map_width, 1600);  // lidar horizontal resolution
        node_.param<int>  ("gap_extractor/2D/range_map_height",                             params_.range_map_height, 1);  // only one row for 2D lidar
        node_.param<int>  ("gap_extractor/2D/map_size",                                     params_.map_size, 5); // meters

        node_.param<float>("gap_extractor/2D/yaw_split_threshold_in_limited_gap_region",    params_.yaw_split_threshold_in_limited_gap_region, M_PI / 6); // 30 degrees
        node_.param<float>("gap_extractor/2D/elev_split_threshold_in_limited_gap_region",   params_.elev_split_threshold_in_limited_gap_region, M_PI / 9); // 20 degrees
        node_.param<int>  ("gap_extractor/2D/min_pixels_in_limited_gap_region",             params_.min_pixels_in_limited_gap_region, 2);
        node_.param<float>("gap_extractor/2D/limited_gap_yaw_span",                         params_.limited_gap_yaw_span, M_PI / 6); // 30 degrees
        node_.param<float>("gap_extractor/2D/limited_gap_elev_span",                        params_.limited_gap_elev_span, M_PI / 6); // 30 degrees
        node_.param<int>  ("gap_extractor/2D/min_pixels_in_limited_subregion",              params_.min_pixels_in_limited_subregion, 2);
    }

    cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (env_type_){
        ROS_INFO("[GapExtractor] Initializing for 3D environment (Velodyne pointcloud)...");
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &GapExtractor::velodyneCallback, this);
    } else{
        ROS_INFO("[GapExtractor] Initializing for 2D environment (LaserScan)...");
        scan2d_sub_ = node_.subscribe("/scan", 1, &GapExtractor::scan2dCallback, this);
        params_.range_map_height = 1; // only one row for 2D lidar
        params_.min_elev = -1.f;
        params_.max_elev =  1.f;
    }

    // initialize range map
    range_map_.azimuth.resize(params_.range_map_height, std::vector<float>(params_.range_map_width, std::numeric_limits<float>::max()));
    range_map_.elevation.resize(params_.range_map_height, std::vector<float>(params_.range_map_width, std::numeric_limits<float>::max()));
    range_map_.range.resize(params_.range_map_height, std::vector<float>(params_.range_map_width, std::numeric_limits<float>::max()));
    // initialize gap masks
    gap_masks_.open.resize(params_.range_map_height, std::vector<uint8_t>(params_.range_map_width, 0));
    gap_masks_.limited.resize(params_.range_map_height, std::vector<uint8_t>(params_.range_map_width, 0));
    gap_masks_.free.resize(params_.range_map_height, std::vector<uint8_t>(params_.range_map_width, 0));

    limited_side_lr_.assign(params_.range_map_height, std::vector<int8_t>(params_.range_map_width, 0));
    limited_side_ud_.assign(params_.range_map_height, std::vector<int8_t>(params_.range_map_width, 0));

    gap_regions_open_.clear();
    gap_regions_limited_.clear();
    gap_regions_free_.clear();
    open_gap_subregions_.clear();
    limited_gap_subregions_.clear();
    free_gap_subregions_.clear();

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

    image_pub_ = node_.advertise<sensor_msgs::Image>("/range_map_image", 1);
    edge_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/detected_edges", 1);
    mask_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/gap_masks", 1);
    subregion_pub_ = node_.advertise<visualization_msgs::MarkerArray>("/gap_subregions", 1);
    gap_candidates_pub_ = node_.advertise<planner_manager::GapCandidates>("/gap_candidates", 1);

    gap_extractor_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::extractGapCallback, this);
    
    visualization_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::visualizationCallback, this);
}

void GapExtractor::pointCloudToRangeMap()
{
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    range_map_.range.assign(H, std::vector<float>(W, NaN));
    range_map_.azimuth.assign(H, std::vector<float>(W, 0.f));
    range_map_.elevation.assign(H, std::vector<float>(W, 0.f));

    if (!cloud_ptr_ || cloud_ptr_->empty())
    {
        ROS_WARN("[GapExtractor] Point cloud is not initialized");
        return;
    }

    // Field of view (radians)
    const float min_azimuth = -M_PI;
    const float max_azimuth =  M_PI;
    const float min_elev = params_.min_elev * M_PI / 180.0f;
    const float max_elev = params_.max_elev * M_PI / 180.0f;

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

    auto write_cell = [&](int v, int u, float r, float th, float ph) {
        float &cell = range_map_.range[v][u];
        if (!std::isfinite(cell) || r < cell) {
            cell = r;
            range_map_.azimuth[v][u]   = th;
            range_map_.elevation[v][u] = ph;
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

        // Store original (unclamped) angles for downstream geometry if needed
        write_cell(v, u, r, th, ph);
    }
}

void GapExtractor::medianFilter()
{
    RangeMap filtered_map = range_map_; 
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            const float rC = range_map_.range[v][u];
            if (!std::isfinite(rC)) continue;

            const int uL = (u - 1 + W) % W;
            const int uR = (u + 1) % W;

            const float rL = range_map_.range[v][uL];
            const float rR = range_map_.range[v][uR];

            if (!std::isfinite(rL) || !std::isfinite(rR)) continue;

            const float dpsi = angDist(range_map_.azimuth[v][uL], range_map_.elevation[v][uL], range_map_.azimuth[v][uR], range_map_.elevation[v][uR]);
            const float rnear = std::min(rL, rR);
            const float gate = edge_params_.a_h + edge_params_.b_h * rnear * std::sin(std::max(0.f, dpsi));

            if (std::fabs(rL - rR) <= gate){
                float a = rL, b = rC, c = rR;
                if (a > b) std::swap(a, b);
                if (b > c) std::swap(b, c);
                if (a > b) std::swap(a, b);
                const float med = b; // median

                if (std::fabs(med - rC) > gate){
                    filtered_map.range[v][u] = med;
                }
            }
        }
    }
    range_map_.range.swap(filtered_map.range); 
}

void GapExtractor::fillTinyHoles()
{
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    for (int v = 0; v < H; ++v) {
        int u = 0;
        while (u < W) {
            if (std::isfinite(range_map_.range[v][u])) {
                ++u;
                continue;
            }

            // start of NaN run
            int u_start = u;
            while (u < W && !std::isfinite(range_map_.range[v][u])) {
                ++u;
            }
            int u_end = u - 1;
            int len = u_end - u_start + 1;

            int uL = (u_start - 1 + W) % W;
            int uR = (u_end + 1) % W;

            float rL = range_map_.range[v][uL];
            float rR = range_map_.range[v][uR];

            if (!std::isfinite(rL) || !std::isfinite(rR)) {
                continue; // cannot fill
            }

            if (len == 1) {
                range_map_.range[v][u_start] = std::min(rL, rR);
            }
            else if (len == 2) {
                range_map_.range[v][u_start]     = rL;
                range_map_.range[v][u_start + 1] = rR;
            }
            // len >= 3 : do nothing
        }
    }
}

void GapExtractor::fetchRangeForCompare(int v, int u, float& r, bool& is_free)
{
    const float x = range_map_.range[v][u];
    if(std::isfinite(x)){
        r = x;
        is_free = false;
    } else {
        r = std::numeric_limits<float>::quiet_NaN();
        is_free = true;
    }
}

void GapExtractor::detectEdges()
{
    std::vector<Edge> edges;
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    static constexpr float z_curv_gate = 30.0f;
std::cout <<std::endl;
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
        const float denom = std::max(1e-3f, dphi * dphi);   

        return std::fabs(z_up - 2.0f * z_mid + z_dn) / denom;
    };

    auto try_pair = [&](int v1, int u1, int v2, int u2, int type){

        const bool horizontal = (type == 0);
        const float a = horizontal ? edge_params_.a_h : edge_params_.a_v;
        const float b = horizontal ? edge_params_.b_h : edge_params_.b_v;
        const float lambda = horizontal ? edge_params_.lambda_h : edge_params_.lambda_v;
        // const float eps_diff = horizontal ? edge_params_.eps_h : edge_params_.eps_v;

        float r1c, r2c; bool free1, free2;
        fetchRangeForCompare(v1, u1, r1c, free1);
        fetchRangeForCompare(v2, u2, r2c, free2);

        if (free1 && free2) return; // both free, no edge

        if (free1 ^ free2) {
            if (horizontal) {
                // pair: (v, u) -> (v, u_next)
                if (!free1 && free2) {
                    // finite -> free  ==> L at (v1,u1)
                    edges.push_back(Edge{v1, u1, type, range_map_.range[v1][u1],
                                        EdgeClass::FU, HEdgeType::L, VEdgeType::NONE});
                } else { // free1 && !free2
                    // free -> finite  ==> R at (v2,u2)
                    edges.push_back(Edge{v2, u2, type, range_map_.range[v2][u2],
                                        EdgeClass::FU, HEdgeType::R, VEdgeType::NONE});
                }
            } else {
                // pair: (v, u) -> (v+1, u)
                if (!free1 && free2) {
                    // finite -> free downward ==> U at (v1,u1)
                    edges.push_back(Edge{v1, u1, type, range_map_.range[v1][u1],
                                        EdgeClass::FU, HEdgeType::NONE, VEdgeType::U});
                } else { // free1 && !free2
                    // free -> finite downward ==> D at (v2,u2)
                    edges.push_back(Edge{v2, u2, type, range_map_.range[v2][u2],
                                        EdgeClass::FU, HEdgeType::NONE, VEdgeType::D});
                }
            }
            return;
        }

        // both finite
        const float dr = r1c - r2c;

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
        // also push the "far" edge, and set their htype/vtype to the same
        const int vb = (r1c > r2c) ? v1 : v2;
        const int ub = (r1c > r2c) ? u1 : u2;
        edges.push_back(Edge{vb, ub, type, range_map_.range[vb][ub], EdgeClass::FF, htype, vtype});
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
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;
    gap_masks.open.resize(H, std::vector<uint8_t>(W, 0));
    gap_masks.limited.resize(H, std::vector<uint8_t>(W, 0));
    gap_masks.free.resize(H, std::vector<uint8_t>(W, 0));

    limited_side_lr_.assign(H, std::vector<int8_t>(W, 0));
    limited_side_ud_.assign(H, std::vector<int8_t>(W, 0));


    // horizontal scan
    std::vector<std::vector<std::tuple<int,EdgeClass,HEdgeType>>> rows(H);
    for (const auto& e : selected_edges_) {
        if (e.type != 0) continue;
        if (e.h_edge_type == HEdgeType::NONE) continue;
        if (e.edge_class != EdgeClass::FU) continue; // only use FU edges for horizontal gaps
        rows[e.v].emplace_back(e.u, e.edge_class, e.h_edge_type);
    }

    for (int v = 0; v < H; ++v){
        auto& edge_list = rows[v];

        bool any_finite = false;
        for (int u = 0; u < W; ++u){
            if (std::isfinite(range_map_.range[v][u])) { any_finite = true; break; }
        }
        if (!any_finite){
            for (int u = 0; u < W; ++u){
                gap_masks.free[v][u] = 1;
            }
            continue; 
        }

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

        // ---- OPEN gaps use FU with L→R only ----
        pair_row_stack(EdgeClass::FU, gap_masks.open);
    }

    // vertical scan
    std::vector<std::vector<std::tuple<int,EdgeClass,VEdgeType>>> cols(W);
    for (const auto& e : selected_edges_) {
        if (e.type != 1) continue;
        if (e.v_edge_type == VEdgeType::NONE) continue;
        if (e.v < v_margin_ || e.v >= H - v_margin_) continue;
        if (e.edge_class != EdgeClass::FU) continue; // only use FU edges for vertical gaps
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
        // open gaps: FU-U and FU-D
        scan_and_pair_col(EdgeClass::FU, VEdgeType::U, VEdgeType::D, gap_masks.open);
        // limited gaps: FU-U and FU-D
        // scan_and_pair_col(EdgeClass::FF, VEdgeType::U, VEdgeType::D, gap_masks.limited);
    }

    buildGapMasks_FromSingleFFEdge(gap_masks.limited);

    gap_masks_.open.clear();
    gap_masks_.limited.clear();
    gap_masks_.free.clear();
    gap_masks_.open = gap_masks.open;
    gap_masks_.limited = gap_masks.limited;
    gap_masks_.free = gap_masks.free;
}

void GapExtractor::buildGapMasks_FromSingleFFEdge(std::vector<std::vector<uint8_t>>& mask_limited){
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    // Ensure mask size and clear to zeros
    if ((int)mask_limited.size() != H) {
        mask_limited.assign(H, std::vector<uint8_t>(W, 0));
    } else {
        for (int v = 0; v < H; ++v) {
            std::fill(mask_limited[v].begin(), mask_limited[v].end(), 0);
        }
    }

    // clear side maps (limited only)
    limited_side_lr_.assign(H, std::vector<int8_t>(W, 0));
    limited_side_ud_.assign(H, std::vector<int8_t>(W, 0));

    auto write_side = [&](std::vector<std::vector<int8_t>>& M, int v, int u, int8_t s){
        if (s == 0) return;
        int8_t &old = M[v][u];
        if (old == 0) old = s;
        else if (old != s) old = 0; // conflict => unknown (conservative)
    };

    for (const auto& e : selected_edges_) {
        if (e.edge_class != EdgeClass::FF) continue;

        int v = e.v;
        int u = e.u;

        if (v < 0 || v >= H) continue;
        u %= W; if (u < 0) u += W;

        // seed limited mask
        mask_limited[v][u] = 1;

        // write side-map (store where "free" is)
        if (e.type == 0) {
            // horizontal FF edge: use h_edge_type
            // obstacle on LEFT => free on RIGHT => bias toward +u
            // So: L => +1, R => -1
            int8_t s = 0;
            if (e.h_edge_type == HEdgeType::L) s = +1;
            else if (e.h_edge_type == HEdgeType::R) s = -1;
            write_side(limited_side_lr_, v, u, s);
        } else if (e.type == 1) {
            // vertical FF edge: use v_edge_type
            // Convention: obstacle UP => free DOWN (toward +v of your indexing) => bias elev to + direction
            // So: U => +1, D => -1
            int8_t s = 0;
            if (e.v_edge_type == VEdgeType::U) s = +1;
            else if (e.v_edge_type == VEdgeType::D) s = -1;
            write_side(limited_side_ud_, v, u, s);
        }
    }
}

void GapExtractor::extractGapRegions()
{
    gap_regions_open_.clear();
    gap_regions_limited_.clear();
    gap_regions_free_.clear();
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    // first only process open gaps
    if (gap_masks_.open.empty() || (int)gap_masks_.open.size() != H) return;
    std::vector<std::vector<uint8_t>> visited(H, std::vector<uint8_t>(W, 0));

    // scan all cells
    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            if (!gap_masks_.open[v][u]) continue; // not a gap cell
            if (visited[v][u]) continue;          // already visited

            GapRegion region;
            // BFS queue
            bfsGapRegion(v, u, gap_masks_.open, visited, region);

            if (region.size >= params_.min_pixels_in_open_gap_region){
                gap_regions_open_.emplace_back(std::move(region));
            }
        }
    }
    // ROS_INFO("[GapExtractor] Extracted %lu open gap regions", gap_regions_open_.size());

    // second, process limited gaps
    if (gap_masks_.limited.empty() || (int)gap_masks_.limited.size() != H) return;
    std::vector<std::vector<uint8_t>> visited_limited(H, std::vector<uint8_t>(W, 0));
    for (int v = 0; v < H; ++v){
        for (int u = 0; u < W; ++u){
            if (!gap_masks_.limited[v][u]) continue; // not a gap cell
            if (visited_limited[v][u]) continue;          // already visited

            GapRegion region;
            // BFS queue
            bfsGapRegion(v, u, gap_masks_.limited, visited_limited, region);
            
            // print elev_span and yaw_span for debug
            // ROS_INFO("Limited gap region: size=%d, yaw_span=%.2f, elev_span=%.2f", region.size, region.yaw_span, region.elev_span);
            // std::cout << std::endl;

            const bool pass_yaw  = (region.yaw_span  >= params_.yaw_split_threshold_in_limited_gap_region);
            const bool pass_elev = (region.elev_span >= params_.elev_split_threshold_in_limited_gap_region);
            const bool pass_pix  = (region.size      >= params_.min_pixels_in_limited_gap_region);
            // make sure serveral pixels and at least one angular span condition is met
            if ((pass_yaw || pass_elev) || pass_pix){
                gap_regions_limited_.emplace_back(std::move(region));
            }
        }
    }
    // ROS_INFO("[GapExtractor] Extracted %lu limited gap components", gap_regions_limited_.size());

    // Third, process free space regions
    // first check the size of gap_masks_.free, only process when > certain ratio
    if (gap_masks_.free.empty() || (int)gap_masks_.free.size() != H) return;
    // check how many free layers' pixels are setted
    int free_layer_count = 0;
    for (int v = 0; v < H; ++v){
        if (gap_masks_.free[v][0] > 0){
            free_layer_count++;
        }
    }
    if (free_layer_count > H / 4){
        // put whole layers with free pixels into one gap region
        GapRegion region;
        region.pixels.clear();
        region.size = 0;
        region.v_min = std::numeric_limits<int>::max();
        region.v_max = std::numeric_limits<int>::min();
        for (int v = 0; v < H; ++v){
            if (gap_masks_.free[v][0] > 0){
                for (int u = 0; u < W; ++u){
                    region.pixels.emplace_back(v, u);
                    region.size++;
                }
                region.v_min = std::min(region.v_min, v);
                region.v_max = std::max(region.v_max, v);
            }
        }
        region.center_yaw = std::numeric_limits<float>::quiet_NaN();
        region.yaw_span = 2.f * static_cast<float>(M_PI);

        // compute elev 
        std::vector<float> el_list;
        el_list.reserve(256);
        for (int v = 0; v < H; ++v){
            if (gap_masks_.free[v][0] > 0){
                el_list.push_back(range_map_.elevation[v][0]);
            }
        }

        if (!el_list.empty()){
            std::vector<float> tmp_e = el_list;
            const size_t n = tmp_e.size();
            const size_t m = n >> 1;
            std::nth_element(tmp_e.begin(), tmp_e.begin() + m, tmp_e.end());
            float med_e = tmp_e[m];
            if ((n & 1) == 0){
                float lower = *std::max_element(tmp_e.begin(), tmp_e.begin() + m);
                med_e = 0.5f * (med_e + lower);
            }
            region.center_elev = med_e;
            region.elev_span = linearSpanRad(el_list);
        }
        gap_regions_free_.emplace_back(std::move(region));
    }
}

void GapExtractor::bfsGapRegion(int v0, int u0, const std::vector<std::vector<uint8_t>>& mask, std::vector<std::vector<uint8_t>>& visited, GapRegion& region){
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;

    std::queue<std::pair<int,int>> q;
    visited[v0][u0] = 1;
    q.emplace(v0, u0);

    // Accumulators for spherical mean and angle spans
    float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
    std::vector<float> az_list;
    std::vector<float> el_list;
    az_list.reserve(256);
    el_list.reserve(256);

    region.pixels.clear();
    region.size = 0;
    region.v_min = std::numeric_limits<int>::max();
    region.v_max = std::numeric_limits<int>::min();

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

        az_list.push_back(yaw);
        el_list.push_back(elev);

        // visit neighbors (8-connectivity)
        for (int dv = -1; dv <= 1; ++dv){
            for (int du = -1; du <= 1; ++du){
                if (dv == 0 && du == 0) continue;
                int vv = v + dv;
                if (vv < 0 || vv >= H) continue;
                int uu = (u + du + W) % W; // wrap around
                if (!mask[vv][uu]) continue; // not a gap cell
                if (visited[vv][uu]) continue;          // already visited
                visited[vv][uu] = 1;
                q.emplace(vv, uu);
            }
        }
    }

    // Compute mean direction
    float center_yaw = 0.f;
    float center_elev = 0.f;

    if(!az_list.empty()){
        const float offset = yawUnwrapOffset(az_list);

        std::vector<float> yaw_s;
        yaw_s.reserve(az_list.size());
        for (float a : az_list){
            yaw_s.push_back(yawShiftedByOffset(a, offset));     //[0,2pi]
        }

        std::vector<float> tmp_y = yaw_s;
        const size_t ny = tmp_y.size();
        const size_t my = ny >> 1;
        std::nth_element(tmp_y.begin(), tmp_y.begin() + my, tmp_y.end());
        float med_y = tmp_y[my];
        if ((ny & 1) == 0){
            float lower = *std::max_element(tmp_y.begin(), tmp_y.begin() + my);
            med_y = 0.5f * (med_y + lower);
        }

        float yaw_rec = med_y + offset;
        const float TWO_PI = 2.0f * static_cast<float>(M_PI);
        while (yaw_rec >= TWO_PI) yaw_rec -= TWO_PI;
        while (yaw_rec < 0.f) yaw_rec += TWO_PI;
        if (yaw_rec > M_PI) yaw_rec -= TWO_PI; // back to [-pi, pi]
        center_yaw = yaw_rec;
    }

    if (!el_list.empty()){
        std::vector<float> tmp_e = el_list;
        const size_t ne = tmp_e.size();
        const size_t me = ne >> 1;
        std::nth_element(tmp_e.begin(), tmp_e.begin() + me, tmp_e.end());
        float med_e = tmp_e[me];
        if ((ne & 1) == 0){
            float lower = *std::max_element(tmp_e.begin(), tmp_e.begin() + me);
            med_e = 0.5f * (med_e + lower);
        }
        center_elev = med_e;
    }

    region.center_yaw = center_yaw;
    region.center_elev = center_elev;
    {
        const float c = std::cos(center_elev);
        region.dir_x = c * std::cos(center_yaw);
        region.dir_y = c * std::sin(center_yaw);
        region.dir_z = std::sin(center_elev);
    }

    // Angular spans
    region.yaw_span = circularSpanRad(az_list);
    region.elev_span = linearSpanRad(el_list);
}

void GapExtractor::splitAllGapRegions(){
    // Split each open gap region into smaller regions based on angular spans
    open_gap_subregions_.clear();
    for (size_t i = 0; i < gap_regions_open_.size(); ++i){
        std::vector<std::vector<GapSubRegion>> subregions;
        splitOpenGapRegion(gap_regions_open_[i], params_.open_gap_yaw_span, params_.open_gap_elev_span, params_.min_pixels_in_subregion, subregions);

        open_gap_subregions_.emplace_back(std::move(subregions));
    }
    // split limited gap regions into smaller regions based on angular spans
    limited_gap_subregions_.clear();
    for (size_t i = 0; i < gap_regions_limited_.size(); ++i){
        std::vector<std::vector<GapSubRegion>> subregions;
        splitLimitedGapRegion(gap_regions_limited_[i], params_.limited_gap_yaw_span, params_.limited_gap_elev_span, params_.min_pixels_in_limited_subregion, subregions);

        limited_gap_subregions_.emplace_back(std::move(subregions));
    }
    // split free gap regions into smaller regions based on angular spans
    free_gap_subregions_.clear();
    for (size_t i = 0; i < gap_regions_free_.size(); ++i){
        std::vector<std::vector<GapSubRegion>> subregions;
        splitFreeGapRegion(gap_regions_free_[i], params_.open_gap_yaw_span, params_.open_gap_elev_span, params_.min_pixels_in_subregion, subregions);

        free_gap_subregions_.emplace_back(std::move(subregions));
    }
    // ROS_INFO("[GapExtractor] Split free gap regions into %lu subregion sets", free_gap_subregions_.size());
}

void GapExtractor::splitOpenGapRegion(const GapRegion& region, float yaw_sub_span, float elev_sub_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions){
    subregions.clear();
    if (region.pixels.empty()) return;

    // first build yaw list and compute unwrap offest 
    std::vector<float> yaw_list;
    yaw_list.reserve(region.pixels.size());
    for (size_t i = 0; i < region.pixels.size(); ++i){
        int v = region.pixels[i].first;
        int u = region.pixels[i].second;
        yaw_list.push_back(range_map_.azimuth[v][u]);
    }
    const float yaw_offset = yawUnwrapOffset(yaw_list);
    const float center_yaw_shifted = yawShiftedByOffset(region.center_yaw, yaw_offset);

    // get local Bounding Box in shifted yaw and elevation
    std::vector<float> yaw_s, elev_s;
    float yaw_min, yaw_max, elev_min, elev_max;
    collectShiftedAnglesAndBBox(region, yaw_offset, yaw_s, elev_s, yaw_min, yaw_max, elev_min, elev_max);
    const float yaw_span  = std::max(0.0f, yaw_max  - yaw_min);
    const float elev_span = std::max(0.0f, elev_max - elev_min);

    // Decide Ny/Ne (capped by thresholds; at least 1 tile each)
    int Ny = (yaw_sub_span  > 1e-6f) ? (int)std::ceil(yaw_span / yaw_sub_span)  : 1;
    int Ne = (elev_sub_span > 1e-6f) ? (int)std::ceil(elev_span / elev_sub_span) : 1;
    if (Ny < 1) Ny = 1;
    if (Ne < 1) Ne = 1;

    subregions.resize(Ny);
    for (int iy = 0; iy < Ny; ++iy){
        subregions[iy].resize(Ne);
    }

    for (int iy = 0; iy < Ny; ++iy){
        for (int ie = 0; ie < Ne; ++ie)
            subregions[iy][ie].type = 0; // open
    }

    // assign pixels to subregions
    assignPixelsToSubregions(region, yaw_s, elev_s, center_yaw_shifted, region.center_elev, yaw_sub_span, elev_sub_span, Ny, Ne, subregions);
 
    // for every subregion, if size < min_pixels, merge to biggest neighbor
    // if all neighbors are empty, keep the subregion
    mergeSmallSubregions(subregions, Ny, Ne, min_pixels);

    // compute mean direction for each subregion
    computeStateForSubregions(subregions);
}

void GapExtractor::collectShiftedAnglesAndBBox(const GapRegion& region, float yaw_offset,
    std::vector<float>& yaw_s_list, std::vector<float>& elev_list,
    float& yaw_min, float& yaw_max, float& elev_min, float& elev_max) const{
    yaw_s_list.clear();
    elev_list.clear();
    yaw_s_list.reserve(region.pixels.size());
    elev_list .reserve(region.pixels.size());

    yaw_min = 1e9f; yaw_max = -1e9f;
    elev_min = 1e9f; elev_max = -1e9f;

    for (const auto& pv : region.pixels){
        const int v = pv.first;
        const int u = pv.second;
        const float ys = yawShiftedByOffset(range_map_.azimuth[v][u], yaw_offset);
        const float ep = range_map_.elevation[v][u];
        yaw_s_list.push_back(ys);
        elev_list.push_back(ep);
        if (ys < yaw_min) yaw_min = ys;
        if (ys > yaw_max) yaw_max = ys;
        if (ep < elev_min) elev_min = ep;
        if (ep > elev_max) elev_max = ep;
    }
}

void GapExtractor::assignPixelsToSubregions(const GapRegion& region, const std::vector<float>& yaw_s, const std::vector<float>& elev_s,
    float center_yaw_shifted, float center_elev, float yaw_sub_span, float elev_sub_span,
    int Ny, int Ne, std::vector<std::vector<GapSubRegion>>& subregions) const{
    const float start_yaw = center_yaw_shifted - 0.5f * Ny * yaw_sub_span;
    const float start_elev = center_elev - 0.5f * Ne * elev_sub_span;

    for (size_t i = 0; i < region.pixels.size(); ++i){
        const int v = region.pixels[i].first;
        const int u = region.pixels[i].second;

        const float ys = yaw_s[i];
        const float ep = elev_s[i];

        int iy = 0, ie = 0;
        if (Ny > 1){
            const float ty = (ys - start_yaw) / yaw_sub_span;
            iy = (int) std::floor(ty);
            if (iy < 0) iy = 0;
            if (iy >= Ny) iy = Ny - 1;
        }
        if (Ne > 1){
            const float te = (ep - start_elev) / elev_sub_span;
            ie = (int) std::floor(te);
            if (ie < 0) ie = 0;
            if (ie >= Ne) ie = Ne - 1;
        }
        GapSubRegion& cell = subregions[iy][ie];
        cell.pixels.emplace_back(v, u);
        cell.size++;
        if (cell.v_min > v) cell.v_min = v;
        if (cell.v_max < v) cell.v_max = v;
    }
}

void GapExtractor::mergeSmallSubregions(std::vector<std::vector<GapSubRegion>>& subregions, int Ny, int Ne, int min_pixels) const{
    auto inGrid = [&](int iy, int ie){return (iy >= 0 && iy < Ny && ie >= 0 && ie < Ne);};
    static const int N[8][2] = {
        {-1, -1}, {-1, 0}, {-1, 1},
        { 0, -1},          { 0, 1},
        { 1, -1}, { 1, 0}, { 1, 1}
    };

    bool changed = true;
    for (int iter = 0; iter < Ny*Ne && changed; ++iter){
        changed = false;
        for (int iy = 0; iy < Ny; ++iy){
            for (int ie = 0; ie < Ne; ++ie){
                GapSubRegion& cell = subregions[iy][ie];
                if (cell.size >= min_pixels || cell.size == 0) continue;

                int biggest_iy = -1, biggest_ie = -1, biggest_size = -1;
                for (int k = 0; k < 8; ++k){
                    const int ny = iy + N[k][0];
                    const int ne = ie + N[k][1];
                    if (!inGrid(ny, ne)) continue;
                    const int sz = subregions[ny][ne].size;
                    if (sz > biggest_size){
                        biggest_size = sz;
                        biggest_iy = ny;
                        biggest_ie = ne;
                    }
                }
                if (biggest_size > 0){
                    // merge to neighbor
                    GapSubRegion& nbr = subregions[biggest_iy][biggest_ie];
                    nbr.pixels.insert(nbr.pixels.end(), cell.pixels.begin(), cell.pixels.end());
                    nbr.size += cell.size;
                    nbr.v_min = std::min(nbr.v_min, cell.v_min);
                    nbr.v_max = std::max(nbr.v_max, cell.v_max);
                    cell.pixels.clear();
                    cell.size = 0;
                    cell.v_min = std::numeric_limits<int>::max();
                    cell.v_max = std::numeric_limits<int>::min();
                    changed = true;
                }
            }
        }
    }
}

void GapExtractor::splitLimitedGapRegion(const GapRegion& region, float min_yaw_span, float min_elev_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions){
    subregions.clear();
    if (region.pixels.empty()) return;

    // build yaw list
    std::vector<float> yaw_list;
    yaw_list.reserve(region.pixels.size());
    for (size_t i = 0; i < region.pixels.size(); ++i){
        yaw_list.push_back(range_map_.azimuth[region.pixels[i].first][region.pixels[i].second]);
    }
    const float yaw_offset = yawUnwrapOffset(yaw_list);
    const float center_yaw_shifted = yawShiftedByOffset(region.center_yaw, yaw_offset);

    // get local Bounding Box in shifted yaw and elevation
    std::vector<float> yaw_s, elev_s;
    float yaw_min, yaw_max, elev_min, elev_max;
    collectShiftedAnglesAndBBox(region, yaw_offset, yaw_s, elev_s, yaw_min, yaw_max, elev_min, elev_max);
    const float yaw_span  = std::max(0.0f, yaw_max  - yaw_min);
    const float elev_span = std::max(0.0f, elev_max - elev_min);

    // decide main axis based on spans
    const bool yaw_is_axis = (yaw_span >= elev_span);
    int Ny = yaw_is_axis ? std::max(1, (int)std::ceil(yaw_span  / min_yaw_span)) : 1;
    int Ne = yaw_is_axis ? 1 : std::max(1, (int)std::ceil(elev_span / min_elev_span));

    subregions.resize(Ny);
    for (int iy = 0; iy < Ny; ++iy){
        subregions[iy].resize(Ne);
    }

    for (int iy = 0; iy < Ny; ++iy){
        for (int ie = 0; ie < Ne; ++ie)
            subregions[iy][ie].type = 1; // limited
    }

    // how many subregions along each axis
    // ROS_INFO("[GapExtractor] split into %lu subregions",  Ny * Ne);

    // assign pixels to subregions
    assignPixelsToSubregions(region, yaw_s, elev_s, center_yaw_shifted, region.center_elev, min_yaw_span, min_elev_span, Ny, Ne, subregions);

    // for every subregion, if size < min_pixels, merge to biggest neighbor
    // if all neighbors are empty, keep the subregion
    mergeSmallSubregions(subregions, Ny, Ne, min_pixels);

    // how many subregions left after merging
    int count_subregions = 0;
    for (int iy = 0; iy < Ny; ++iy){
        for (int ie = 0; ie < Ne; ++ie){
            if (subregions[iy][ie].size > 0) count_subregions++;
        }
    }
    // ROS_INFO("[GapExtractor] %d subregions left after merging",  count_subregions);
    // std::cout << std::endl;

    // compute mean direction for each subregion
    computeStateForSubregions(subregions);
}

void GapExtractor::splitFreeGapRegion(const GapRegion& region, float yaw_sub_span, float elev_sub_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions){
    subregions.clear();
    if (region.pixels.empty()) return;

    float elev_min = 1e9f, elev_max = -1e9f;
    for (const auto& pv : region.pixels){
        const int v = pv.first;
        const int u = pv.second;
        const float ep = range_map_.elevation[v][u];
        if (ep < elev_min) elev_min = ep;
        if (ep > elev_max) elev_max = ep;
    }
    const float elev_span = std::max(0.0f, elev_max - elev_min);

    const float TWO_PI = 2.f * static_cast<float>(M_PI);
    int Ny = (yaw_sub_span  > 1e-6f) ? (int)std::ceil(TWO_PI / yaw_sub_span)  : 1;
    int Ne = (elev_sub_span > 1e-6f) ? (int)std::ceil(elev_span / elev_sub_span) : 1;
    if (Ny < 1) Ny = 1;
    if (Ne < 1) Ne = 1;

    subregions.resize(Ny);
    for (int iy = 0; iy < Ny; ++iy) subregions[iy].resize(Ne);

    for (int iy = 0; iy < Ny; ++iy){
        for (int ie = 0; ie < Ne; ++ie)
            subregions[iy][ie].type = 2; // free
    }

    const float start_yaw  = 0.0f;    
    const float start_elev = elev_min;

    for (const auto& pv : region.pixels){
        const int v = pv.first;
        const int u = pv.second;

        const float yaw  = range_map_.azimuth  [v][u];
        const float elev = range_map_.elevation[v][u];

        // yaw_s ∈ [0, 2π)
        const float yaw_s = yawShiftedByOffset(yaw, 0.0f);

        int iy = 0, ie = 0;
        if (Ny > 1){
            const float ty = (yaw_s - start_yaw) / yaw_sub_span;
            iy = (int)std::floor(ty);
            if (iy < 0)    iy = 0;
            if (iy >= Ny)  iy = Ny - 1;
        }
        if (Ne > 1){
            const float te = (elev - start_elev) / elev_sub_span;
            ie = (int)std::floor(te);
            if (ie < 0)    ie = 0;
            if (ie >= Ne)  ie = Ne - 1;
        }

        auto& cell = subregions[iy][ie];
        cell.pixels.emplace_back(v, u);
        cell.size++;
        if (cell.v_min > v) cell.v_min = v;
        if (cell.v_max < v) cell.v_max = v;
    }
    mergeSmallSubregions(subregions, Ny, Ne, min_pixels);
    computeStateForSubregions(subregions);
}

void GapExtractor::computeStateForCell(GapSubRegion& cell) const {
    if (cell.size == 0) return;

    double r_sum = 0.0;

    std::vector<float> yz; yz.reserve(cell.pixels.size());
    std::vector<float> el; el .reserve(cell.pixels.size());

    for (const auto& pv : cell.pixels){
        const int v = pv.first;
        const int u = pv.second;
        float r = range_map_.range[v][u];
        if (!std::isfinite(r)){
            r = params_.map_size; // max range
        }
        r_sum += (double) r;

        yz.push_back(range_map_.azimuth[v][u]);
        el.push_back(range_map_.elevation[v][u]);
    }

    float center_yaw = 0.f;
    if (!yz.empty()){
        const float offset = yawUnwrapOffset(yz);

        std::vector<float> yaw_s;
        yaw_s.reserve(yz.size());
        for (float a : yz){
            yaw_s.push_back(yawShiftedByOffset(a, offset));     //[0,2pi]
        }

        std::vector<float> tmp_y = yaw_s;
        const size_t n = tmp_y.size();
        const size_t m = n >> 1;
        std::nth_element(tmp_y.begin(), tmp_y.begin() + m, tmp_y.end());
        float med_y = tmp_y[m];
        if ((n & 1) == 0){
            float lower = *std::max_element(tmp_y.begin(), tmp_y.begin() + m);
            med_y = 0.5f * (med_y + lower);
        }
        float yaw_rec = med_y + offset;
        const float TWO_PI = 2.0f * static_cast<float>(M_PI);
        while (yaw_rec >= TWO_PI) yaw_rec -= TWO_PI;
        while (yaw_rec < 0.f) yaw_rec += TWO_PI;
        if (yaw_rec > M_PI) yaw_rec -= TWO_PI; // back to [-pi, pi]
        center_yaw = yaw_rec;
    }

    float center_elev = 0.f;
    if (!el.empty()){
        std::vector<float> tmp_e = el;
        const size_t n = tmp_e.size();
        const size_t m = n >> 1;
        std::nth_element(tmp_e.begin(), tmp_e.begin() + m, tmp_e.end());
        float med_e = tmp_e[m];
        if ((n & 1) == 0){
            float lower = *std::max_element(tmp_e.begin(), tmp_e.begin() + m);
            med_e = 0.5f * (med_e + lower);
        }
        center_elev = med_e;
    }

    cell.center_yaw = center_yaw;
    cell.center_elev = center_elev;

    cell.yaw_span = circularSpanRad(yz);
    cell.elev_span = linearSpanRad(el);

    cell.range_mean = (cell.size > 0) ? (float)(r_sum / (double)cell.size) : 0.0f;

    // ---------------- LIMITED bias (use side maps) ----------------
    if (cell.type == 1 && cell.size > 0) {

        int lr_sum = 0, lr_cnt = 0;
        int ud_sum = 0, ud_cnt = 0;

        for (const auto& pv : cell.pixels){
            const int v = pv.first;
            const int u = pv.second;

            if (v < 0 || v >= (int)limited_side_lr_.size()) continue;
            if (u < 0 || u >= (int)limited_side_lr_[0].size()) continue;

            int8_t lr = limited_side_lr_[v][u];
            int8_t ud = limited_side_ud_[v][u];

            if (lr != 0) { lr_sum += (int)lr; lr_cnt++; }
            if (ud != 0) { ud_sum += (int)ud; ud_cnt++; }
        }

        auto sgn = [&](int x)->int { return (x > 0) - (x < 0); };

        const int lr_dir = sgn(lr_sum);
        const int ud_dir = sgn(ud_sum);

        const float kYaw  = params_.limited_gap_bias_yaw;
        const float kElev = params_.limited_gap_bias_elev;

        if (lr_dir != 0 && cell.yaw_span > 1e-6f) {
            cell.yaw_bias = lr_dir * kYaw;
            // float dyaw = lr_dir * kYaw;
            // // wrap back to [-pi, pi]
            // cell.center_yaw += dyaw;
            // while (cell.center_yaw >  (float)M_PI) cell.center_yaw -= 2.f * (float)M_PI;
            // while (cell.center_yaw < -(float)M_PI) cell.center_yaw += 2.f * (float)M_PI;
        }

        if (ud_dir != 0 && cell.elev_span > 1e-6f) {
            cell.elev_bias = ud_dir * kElev;
            // float de = ud_dir * kElev;

            // // clamp to lidar elevation range
            // const float min_e = params_.min_elev * (float)M_PI / 180.f;
            // const float max_e = params_.max_elev * (float)M_PI / 180.f;
            // cell.center_elev = std::max(min_e, std::min(max_e, cell.center_elev + de));
        }
    }
}

void GapExtractor::computeStateForSubregions(std::vector<std::vector<GapSubRegion>>& subregions) const{
    const int Ny = (int)subregions.size();
    const int Ne = Ny ? (int)subregions[0].size() : 0;
    for (int iy = 0; iy < Ny; ++iy){
        for (int ie = 0; ie < Ne; ++ie){
            if (subregions[iy][ie].size == 0) continue;
            computeStateForCell(subregions[iy][ie]);
        }
    }
}

void GapExtractor::checkGoalStatus(Eigen::Vector3d goal_pos){
    const int H = params_.range_map_height;
    const int W = params_.range_map_width;
    if (range_map_.range.empty() || (int)range_map_.range.size() != H || (int)range_map_.range[0].size() != W){
        goal_status_ = GoalStatus::OUT_OF_VIEW;
        return;
        // range map not ready
    }
    if (!base_to_odom_ptr_) {
        goal_status_ = GoalStatus::OUT_OF_VIEW;
        return;
    }
    if (!lidar_to_base_ptr_) {
        goal_status_ = GoalStatus::OUT_OF_VIEW;
        return;
    }
    // transferm goal pos from odom to scan frame
    Eigen::Affine3d T_odom_base = tf2::transformToEigen(*base_to_odom_ptr_);
    Eigen::Affine3d T_base_scan = tf2::transformToEigen(*lidar_to_base_ptr_);
    Eigen::Affine3d T_odom_scan = T_odom_base * T_base_scan;
    Eigen::Affine3d T_scan_odom = T_odom_scan.inverse();

    Eigen::Vector4d goal_odom_h(goal_pos[0], goal_pos[1], goal_pos[2], 1.0);
    Eigen::Vector4d goal_scan_h = T_scan_odom.matrix() * goal_odom_h;
    Eigen::Vector3d goal_scan = goal_scan_h.head<3>();

    const double x = goal_scan.x();
    const double y = goal_scan.y();
    const double z = goal_scan.z();
    const double r_goal = std::sqrt(x*x + y*y + z*z);
    const double eps_range = 1e-32;
    
    if (!std::isfinite(r_goal) || r_goal < 1e-3){
        goal_status_ = GoalStatus::FREE;
        return;
    }
    if (r_goal > (double) params_.map_size + eps_range){
        goal_status_ = GoalStatus::OUT_OF_VIEW;
        return;
    } 

    // Field of view (radians)
    const float min_azimuth = -M_PI;
    const float max_azimuth =  M_PI;
    const float min_elev = params_.min_elev * M_PI / 180.0f;
    const float max_elev = params_.max_elev * M_PI / 180.0f;

    const float azimuth_res = (max_azimuth - min_azimuth) / static_cast<float>(W);
    const float elev_res    = (max_elev    - min_elev)    / static_cast<float>(H);
    const float inv_az_res  = 1.f / azimuth_res;
    const float inv_el_res  = 1.f / elev_res;
    const float EPS = 1e-6f;
    const float two_pi = 2.f * M_PI;

    float th = std::atan2((float)y, (float)x);                     // [-pi, pi]
    float ph = std::atan2((float)z, std::sqrt((float)(x*x + y*y))); // [-pi/2, pi/2]

    float ph_c = std::min(std::max(ph, min_elev + EPS), max_elev - EPS);

    float th_w = th;
    while (th_w <  min_azimuth) th_w += two_pi;
    while (th_w >= max_azimuth) th_w -= two_pi;
    th_w = std::min(std::max(th_w, min_azimuth + EPS), max_azimuth - EPS);

    // Compute floating indices of the "goal pixel center"
    const float v_float = (ph_c - (min_elev + 0.5f * elev_res)) * inv_el_res;
    const float u_float = (th_w - (min_azimuth + 0.5f * azimuth_res)) * inv_az_res;

    int v0 = (int)std::floor(v_float);
    int v1 = v0 + 1;
    int u0 = (int)std::floor(u_float);
    int u1 = u0 + 1;

    v0 = std::max(0, std::min(H-1, v0));
    v1 = std::max(0, std::min(H-1, v1));
    u0 = (u0 % W + W) % W;
    u1 = (u1 % W + W) % W;

    struct Pix { int v, u; };
    std::vector<Pix> neigh;
    neigh.reserve(4);
    neigh.push_back({v0, u0});
    neigh.push_back({v0, u1});
    neigh.push_back({v1, u0});
    neigh.push_back({v1, u1});

    bool any_echo = false;
    bool any_block_before_goal = false;
    bool all_goal_before_obstacle = true;

    for (const auto& px : neigh) {
        float r_echo = range_map_.range[px.v][px.u];

        if (!std::isfinite(r_echo)) continue;  // no obstacle echo

        any_echo = true;

        if (r_goal > (double)r_echo + eps_range) {
            any_block_before_goal = true;
        }
        if (!(r_goal < (double)r_echo - eps_range)) {
            all_goal_before_obstacle = false;
        }
    }
    // check gap membership
    bool masks_ready = (!gap_masks_.open.empty() &&
                        (int)gap_masks_.open.size() == H &&
                        (int)gap_masks_.open[0].size() == W &&
                        !gap_masks_.free.empty() &&
                        (int)gap_masks_.free.size() == H &&
                        (int)gap_masks_.free[0].size() == W);

    bool in_gap_open_free = false;
    if (masks_ready) {
        in_gap_open_free = true;  
        for (const auto& px : neigh) {
            bool is_gap_pixel =
                (gap_masks_.open[px.v][px.u] > 0) ||
                (gap_masks_.free[px.v][px.u] > 0);

            if (!is_gap_pixel) {
                in_gap_open_free = false;
                break;
            }
        }
    }
    if (!any_echo) {
        // LOS free up to max range along these directions
        goal_status_ = in_gap_open_free ? GoalStatus::FREE : GoalStatus::LIMITED;
        return;
    }
    if (any_block_before_goal) {
        // There is at least one ray where an obstacle is closer than the goal
        goal_status_ = GoalStatus::BLOCKED;
        return;
    }
    if (all_goal_before_obstacle) {
        // LOS is free (goal in front of obstacles for all rays with echo)
        goal_status_ = in_gap_open_free ? GoalStatus::FREE : GoalStatus::LIMITED;
        return;
    }

    // Mixed / borderline case: some rays ~ on obstacle surface.
    // Use conservative choice: treat as BLOCKED.
    goal_status_ = GoalStatus::BLOCKED;
    return;
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
    extractGapRegions();
    // split gap regions into subregions
    splitAllGapRegions();
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

void GapExtractor::scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg){
    // Process the 2D laser scan data
    if (msg->ranges.empty())
    {
        ROS_WARN("[GapExtractor] Received empty laser scan data");
        return;
    }
    // process laser scan to pcl::PointCloud<pcl::PointXYZ>::Ptr
    if (!cloud_ptr_){
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    cloud_ptr_->clear();
    double angle = msg->angle_min;
    for (const float& r : msg->ranges){
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max){
            angle += msg->angle_increment;
            continue;
        }
        pcl::PointXYZ p;
        p.x = r * std::cos(angle);
        p.y = r * std::sin(angle);
        p.z = 0.0f;
        cloud_ptr_->points.push_back(p);
        angle += msg->angle_increment;
    }
    cloud_ptr_->width = cloud_ptr_->points.size();
    cloud_ptr_->height = 1;
    cloud_ptr_->is_dense = false;
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
    img_msg.height = params_.range_map_height;
    img_msg.width = params_.range_map_width;
    img_msg.encoding = "mono8";
    img_msg.is_bigendian = false;
    img_msg.step = params_.range_map_width;
    img_msg.data.resize(params_.range_map_height * params_.range_map_width);

    float min_val = std::numeric_limits<float>::max();
    float max_val = 0;
    for (int v = 0; v < params_.range_map_height; ++v)
        for (int u = 0; u < params_.range_map_width; ++u)
            if (range_map_.range[v][u] < std::numeric_limits<float>::max()) {
                min_val = std::min(min_val, range_map_.range[v][u]);
                max_val = std::max(max_val, range_map_.range[v][u]);
            }
    if (min_val >= max_val) min_val = 0, max_val = min_val + 1.0f;

    for (int v = 0; v < params_.range_map_height; ++v)
    {
        for (int u = 0; u < params_.range_map_width; ++u)
        {
            float val = range_map_.range[v][u];
            uint8_t pixel = 0;
            if (val < std::numeric_limits<float>::max())
                pixel = static_cast<uint8_t>(255.0f * (val - min_val) / (max_val - min_val));
            img_msg.data[v * params_.range_map_width + u] = pixel;
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
        // skip edges with invalid range (could be NaN for free)
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
    marker_array.markers.push_back(
        maskToMarkerPoints(gap_masks_.free, range_map_, 3.0f, 0.1f, 0.2f, 0.5f, 2)
    );
    mask_pub_.publish(marker_array);
}

void GapExtractor::publishSubGapRegions() {
    visualization_msgs::MarkerArray arr;

    // ---------- OPEN subregions (magenta) ----------
    visualization_msgs::Marker m_open;
    m_open.header.frame_id = "scan";
    m_open.header.stamp    = ros::Time::now();
    m_open.ns   = "gap_subregions";
    m_open.id   = 0;  // unique id for OPEN
    m_open.type = visualization_msgs::Marker::LINE_LIST;
    m_open.action = visualization_msgs::Marker::ADD;
    m_open.scale.x = 0.02f;        // line width
    m_open.color.r = 1.0f;         // magenta
    m_open.color.g = 0.0f;
    m_open.color.b = 1.0f;
    m_open.color.a = 1.0f;

    geometry_msgs::Point origin;    // (0,0,0)
    int seg_open = 0;

    for (size_t i = 0; i < open_gap_subregions_.size(); ++i){
        const auto& sub = open_gap_subregions_[i];
        for (size_t iy = 0; iy < sub.size(); ++iy){
            for (size_t ie = 0; ie < sub[iy].size(); ++ie){
                const auto& cell = sub[iy][ie];
                if (cell.size == 0) continue;

                const float r  = cell.range_mean;
                const float th = cell.center_yaw;
                const float ph = cell.center_elev;

                geometry_msgs::Point p;
                p.x = r * std::cos(ph) * std::cos(th);
                p.y = r * std::cos(ph) * std::sin(th);
                p.z = r * std::sin(ph);

                m_open.points.push_back(origin);
                m_open.points.push_back(p);
                ++seg_open;
            }
        }
    }

    if (seg_open == 0) {
        m_open.action = visualization_msgs::Marker::DELETE;
        m_open.points.clear();
    }
    arr.markers.push_back(m_open);

    // ---------- LIMITED subregions (cyan) ----------
    visualization_msgs::Marker m_lim;
    m_lim.header.frame_id = "scan";
    m_lim.header.stamp    = ros::Time::now();
    m_lim.ns   = "gap_subregions";
    m_lim.id   = 1;  // unique id for LIMITED
    m_lim.type = visualization_msgs::Marker::LINE_LIST;
    m_lim.action = visualization_msgs::Marker::ADD;
    m_lim.scale.x = 0.02f;         // line width
    m_lim.color.r = 0.0f;          // cyan
    m_lim.color.g = 1.0f;
    m_lim.color.b = 1.0f;
    m_lim.color.a = 1.0f;

    int seg_lim = 0;

    for (size_t i = 0; i < limited_gap_subregions_.size(); ++i){
        const auto& sub = limited_gap_subregions_[i];
        for (size_t iy = 0; iy < sub.size(); ++iy){
            for (size_t ie = 0; ie < sub[iy].size(); ++ie){
                const auto& cell = sub[iy][ie];
                if (cell.size == 0) continue;

                const float r  = cell.range_mean;
                float th = cell.center_yaw + cell.yaw_bias;
                float ph = cell.center_elev + cell.elev_bias;

                while (th >  (float)M_PI) th -= 2.f * (float)M_PI;
                while (th < -(float)M_PI) th += 2.f * (float)M_PI;
                const float min_e = params_.min_elev * (float)M_PI / 180.f;
                const float max_e = params_.max_elev * (float)M_PI / 180.f;
                ph = std::max(min_e, std::min(max_e, ph));

                geometry_msgs::Point p;
                p.x = r * std::cos(ph) * std::cos(th);
                p.y = r * std::cos(ph) * std::sin(th);
                p.z = r * std::sin(ph);

                m_lim.points.push_back(origin);
                m_lim.points.push_back(p);
                ++seg_lim;
            }
        }
    }

    if (seg_lim == 0) {
        m_lim.action = visualization_msgs::Marker::DELETE;
        m_lim.points.clear();
    }
    arr.markers.push_back(m_lim);

    // ---------- FREE subregions (orange) ----------
    visualization_msgs::Marker m_free;
    m_free.header.frame_id = "scan";
    m_free.header.stamp    = ros::Time::now();
    m_free.ns   = "gap_subregions";
    m_free.id   = 2;  // unique id for FREE
    m_free.type = visualization_msgs::Marker::LINE_LIST;
    m_free.action = visualization_msgs::Marker::ADD;
    m_free.scale.x = 0.02f;         // line width
    m_free.color.r = 1.0f;          // orange
    m_free.color.g = 0.65f;
    m_free.color.b = 0.0f;
    m_free.color.a = 1.0f;

    int seg_free = 0;

    for (size_t i = 0; i < free_gap_subregions_.size(); ++i){
        const auto& sub = free_gap_subregions_[i];
        for (size_t iy = 0; iy < sub.size(); ++iy){
            for (size_t ie = 0; ie < sub[iy].size(); ++ie){
                const auto& cell = sub[iy][ie];
                if (cell.size == 0) continue;

                const float r  = cell.range_mean;
                const float th = cell.center_yaw;   
                const float ph = cell.center_elev;

                geometry_msgs::Point p;
                p.x = r * std::cos(ph) * std::cos(th);
                p.y = r * std::cos(ph) * std::sin(th);
                p.z = r * std::sin(ph);

                m_free.points.push_back(origin);
                m_free.points.push_back(p);
                ++seg_free;
            }
        }
    }

    if (seg_free == 0) {
        m_free.action = visualization_msgs::Marker::DELETE;
        m_free.points.clear();
    }
    arr.markers.push_back(m_free);

    // ---- publish ----
    subregion_pub_.publish(arr);
}

void GapExtractor::publishGapCandidates(){
    planner_manager::GapCandidates out;
    out.header.stamp    = ros::Time::now();
    out.header.frame_id = "scan";

    auto emit_from = [&](const std::vector<std::vector<std::vector<GapSubRegion>>>& sub3d)
    {
        for (size_t i = 0; i < sub3d.size(); ++i) {
            const auto& rows = sub3d[i];
            for (size_t j = 0; j < rows.size(); ++j) {
                const auto& cells = rows[j];
                for (size_t k = 0; k < cells.size(); ++k) {
                    const GapSubRegion& cell = cells[k];
                    if (cell.size == 0) continue;

                    planner_manager::GapCandidate c;
                    c.type        = cell.type;              // 0=open, 1=limited, 2=free
                    c.center_yaw  = cell.center_yaw;
                    c.center_elev = cell.center_elev;
                    c.yaw_bias    = cell.yaw_bias;
                    c.elev_bias   = cell.elev_bias;
                    c.yaw_span    = cell.yaw_span;
                    c.elev_span   = cell.elev_span;
                    c.size        = cell.size;
                    c.range_mean  = cell.range_mean;

                    const float cy = std::cos(cell.center_elev);
                    geometry_msgs::Vector3 v;
                    v.x = cell.range_mean * cy * std::cos(cell.center_yaw);
                    v.y = cell.range_mean * cy * std::sin(cell.center_yaw);
                    v.z = cell.range_mean * std::sin(cell.center_elev);
                    c.dir = v;

                    out.items.push_back(c);
                }
            }
        }
    };

    emit_from(open_gap_subregions_);
    emit_from(limited_gap_subregions_);
    emit_from(free_gap_subregions_);

    gap_candidates_pub_.publish(out);
}

void GapExtractor::visualizationCallback(const ros::TimerEvent &e)
{
    publishRangeMapAsImage();
    publishEdges();
    publishMasks();
    publishSubGapRegions();
    publishGapCandidates();
}