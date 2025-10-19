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

    gap_extractor_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::extractGapCallback, this);
    
    visualization_timer_ = node_.createTimer(ros::Duration(0.1), &GapExtractor::visualizationCallback, this);
}

void GapExtractor::pointCloudToRangeMap()
{
    range_map_.azimuth.assign(range_map_height_, std::vector<float>(range_map_width_, 0.f));
    range_map_.elevation.assign(range_map_height_, std::vector<float>(range_map_width_, 0.f));
    const float NaN = std::numeric_limits<float>::quiet_NaN();
    range_map_.range.assign(range_map_height_, std::vector<float>(range_map_width_, NaN));

    if (!cloud_ptr_)
    {
        ROS_WARN("[GapExtractor] Point cloud is not initialized");
        return;
    }

    const float min_azimuth = -M_PI;
    const float max_azimuth = M_PI;
    const float min_elev = -30.67f * M_PI / 180.0f;
    const float max_elev = 20.67f * M_PI / 180.0f;
    const float azimuth_res = (max_azimuth - min_azimuth) / range_map_width_;
    const float elev_res = (max_elev - min_elev) / range_map_height_;

    for (int v = 0; v < range_map_height_; ++v) {
        const float phi_v = min_elev + (v + 0.5f) * elev_res;
        for (int u = 0; u < range_map_width_; ++u) {
            const float th_u = min_azimuth + (u + 0.5f) * azimuth_res;
            range_map_.azimuth[v][u]   = th_u;
            range_map_.elevation[v][u] = phi_v;
        }
    }

    // points are in scan frame, first transform to base frame then to odom frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_odom_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    const Eigen::Matrix4f T_base_odom_mat = T_base_odom.matrix().cast<float>();
    pcl::transformPointCloud(*cloud_ptr_, *cloud_odom_ptr, T_lidar_base_mat_ * T_base_odom_mat);
    // pop out all the points on the ground (z < 0.01)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_odom_ptr->points)
    {
        if (point.z > 0.01)
        {
            cloud_ground_ptr->points.push_back(point);
        }
    }

    for (const auto& pt : cloud_ground_ptr->points) {
        const float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (r < 1e-3f) continue;
        const float az  = std::atan2(pt.y, pt.x);  // [-pi, pi]
        const float elv = std::asin(pt.z / r);     // [-pi/2, pi/2]

        if (elv < min_elev || elv > max_elev) continue;

        int u = static_cast<int>(std::floor((az  - min_azimuth) / azimuth_res));
        int v = static_cast<int>(std::floor((elv - min_elev)    / elev_res));
        // clamp & seam wrap
        if (u < 0) u = 0; if (u >= range_map_width_)  u = range_map_width_ - 1;
        if (v < 0) v = 0; if (v >= range_map_height_) v = range_map_height_ - 1;

        float& cell = range_map_.range[v][u];
        if (!std::isfinite(cell) || r < cell) cell = r; // 选择最近距离
    }
}

void GapExtractor::medianFilter()
{
        RangeMap filtered_map = range_map_; 
    const int H = range_map_height_;
    const int W = range_map_width_;
    for(int v=1; v<H-1; ++v){
        for(int u=0; u<W; ++u){
            float xs[9];
            int n=0;
            auto push = [&](int vv,int uu){
                float r = range_map_.range[vv][uu];
                if (std::isfinite(r)) xs[n++]=r;
            };
            push(v,u);
            for(int dv=-1; dv<=1; ++dv){
                for(int du=-1; du<=1; ++du){
                    if(!dv && !du) continue;
                    int vv=v+dv, uu=(u+du+W)%W;
                    if(vv<0||vv>=H) continue;
                    push(vv,uu);
                }
            }
            if(n>=3){ std::nth_element(xs, xs+n/2, xs+n); filtered_map.range[v][u]=xs[n/2]; }
        }
    }
    range_map_.range.swap(filtered_map.range);
}

void GapExtractor::fillTinyHoles()
{
    RangeMap filtered_map = range_map_; 
    const int H = range_map_height_;
    const int W = range_map_width_;
    for(int v=1; v<H-1; ++v){
        for(int u=0; u<W; ++u){
            if(std::isfinite(range_map_.range[v][u])) continue;
            float sum=0; int cnt=0;
            for(int dv=-1; dv<=1; ++dv){
                for(int du=-1; du<=1; ++du){
                    if(!dv && !du) continue;
                    int vv=v+dv, uu=(u+du+W)%W; if(vv<0||vv>=H) continue;
                    float r=range_map_.range[vv][uu]; if(std::isfinite(r)){ sum+=r; ++cnt; }
                }
            }
            if(cnt>=6) filtered_map.range[v][u]=sum/cnt; // 小孔补洞，大孔保持 NaN
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
    const float a = 0.08f;
    const float b = 0.02f;
    const float lambda = 0.5f;
    const float eps_diff = 1e-3f;

    auto try_pair = [&](int v1, int u1, int v2, int u2, int type){
        float r1_fetched, r2_fetched;
        bool is_unknown1, is_unknown2;
        fetchRangeForCompare(v1, u1, 100.0f, r1_fetched, is_unknown1);
        fetchRangeForCompare(v2, u2, 100.0f, r2_fetched, is_unknown2);
        if (is_unknown1 && is_unknown2) return; // both unknown, no edge

        HEdgeType htype = HEdgeType::NONE;
        VEdgeType vtype = VEdgeType::NONE;
        const float dr = r1_fetched - r2_fetched;
        if (type == 0) {                                    // horizontal edge
            if      (dr < -eps_diff) htype = HEdgeType::L;  // 
            else if (dr >  eps_diff) htype = HEdgeType::R;  // 
            // else NONE
        } else {                                            // vertical edge
            if      (dr < -eps_diff) vtype = VEdgeType::U;  // 
            else if (dr >  eps_diff) vtype = VEdgeType::D;  // 
            // else NONE
        }

        // both have finite ranges
        if (!is_unknown1 && !is_unknown2){
            const float angle_dist = angDist(range_map_.azimuth[v1][u1], range_map_.elevation[v1][u1],
                                            range_map_.azimuth[v2][u2], range_map_.elevation[v2][u2]);
            const float r_min = std::min(r1_fetched, r2_fetched);
            const float threshold = a + b * r_min * std::sin(angle_dist);
    
            if (std::fabs(r1_fetched - r2_fetched) <= threshold) return;
    
            const float r_near = r_min;
            const float r_far = std::max(r1_fetched, r2_fetched);
    
            if (r_far * std::cos(angle_dist) <= r_near + lambda*r_near*std::sin(angle_dist)) return;
            
            edges.push_back(Edge{v1, u1, type, EdgeClass::FF, htype, vtype});
        }
        else if (is_unknown1 != is_unknown2){
            // one finite, one unknown
            edges.push_back(Edge{v1, u1, type, EdgeClass::FU, htype, vtype});
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
        for(int v=0; v<H-1; ++v){
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
        scan_and_pair(EdgeClass::FF, HEdgeType::L, HEdgeType::R, gap_masks.open);
        // limited gaps: FU-L and FU-R
        scan_and_pair(EdgeClass::FU, HEdgeType::L, HEdgeType::R, gap_masks.limited);
    }

    // vertical scan
    std::vector<std::vector<std::tuple<int,EdgeClass,VEdgeType>>> cols(W);
    for (const auto& e : selected_edges_) {
        if (e.type != 1) continue;
        if (e.v_edge_type == VEdgeType::NONE) continue;
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
        scan_and_pair_col(EdgeClass::FF, VEdgeType::U, VEdgeType::D, gap_masks.open);
        // limited gaps: FU-U and FU-D
        scan_and_pair_col(EdgeClass::FU, VEdgeType::U, VEdgeType::D, gap_masks.limited);
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
}

inline float GapExtractor::angDist(float th1, float ph1, float th2, float ph2)
{
    float dth = th1 - th2;
    // wrap to [-pi, pi]
    if (dth > M_PI) dth -= 2 * M_PI;
    if (dth < -M_PI) dth += 2 * M_PI;
    return std::acos(std::sin(ph1)*std::sin(ph2) + std::cos(ph1)*std::cos(ph2)*std::cos(dth));
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

    // 找到最小和最大距离用于归一化
    float min_val = std::numeric_limits<float>::max();
    float max_val = 0;
    for (int v = 0; v < range_map_height_; ++v)
        for (int u = 0; u < range_map_width_; ++u)
            if (range_map_.range[v][u] < std::numeric_limits<float>::max()) {
                min_val = std::min(min_val, range_map_.range[v][u]);
                max_val = std::max(max_val, range_map_.range[v][u]);
            }
    if (min_val >= max_val) min_val = 0, max_val = min_val + 1.0f;

    // 填充图像数据
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

void GapExtractor::visualizationCallback(const ros::TimerEvent &e)
{
    publishRangeMapAsImage();
}