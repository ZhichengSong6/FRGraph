#include "planner_manager.h"

namespace {

inline double wrapAngleRad(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

inline double clampDouble(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

static std::vector<double> buildUniformSamplesInclusive(double lo, double hi, int n) {
    std::vector<double> vals;
    if (n <= 0) return vals;
    vals.resize(n);
    if (n == 1) {
        vals[0] = 0.5 * (lo + hi);
        return vals;
    }
    for (int i = 0; i < n; ++i) {
        vals[i] = lo + (hi - lo) * double(i) / double(n - 1);
    }
    return vals;
}

static std::vector<double> buildCenteredSamplesDeg(double center_rad,
                                                   double half_span_deg,
                                                   double step_deg,
                                                   double lower_bound_rad,
                                                   double upper_bound_rad,
                                                   bool wrap_yaw) {
    std::vector<double> vals;
    const double step = step_deg * M_PI / 180.0;
    const double half = half_span_deg * M_PI / 180.0;
    const int n_side = static_cast<int>(std::round(half / step));

    for (int k = -n_side; k <= n_side; ++k) {
        double v = center_rad + k * step;
        if (wrap_yaw) {
            v = wrapAngleRad(v);
        } else {
            v = clampDouble(v, lower_bound_rad, upper_bound_rad);
        }
        vals.push_back(v);
    }

    std::sort(vals.begin(), vals.end());
    vals.erase(std::unique(vals.begin(), vals.end(),
                           [](double a, double b){ return std::abs(a - b) < 1e-9; }),
               vals.end());
    return vals;
}

static Eigen::Matrix3d rpyToRotation(double yaw, double roll, double pitch) {
    const double cy = std::cos(yaw),   sy = std::sin(yaw);
    const double cr = std::cos(roll),  sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);

    Eigen::Matrix3d R;
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R(0,0) = cy*cp;
    R(0,1) = cy*sp*sr - sy*cr;
    R(0,2) = cy*sp*cr + sy*sr;

    R(1,0) = sy*cp;
    R(1,1) = sy*sp*sr + cy*cr;
    R(1,2) = sy*sp*cr - cy*sr;

    R(2,0) = -sp;
    R(2,1) = cp*sr;
    R(2,2) = cp*cr;
    return R;
}

struct Pose3DEvalResult {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    Eigen::Vector3d vertex = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double yaw = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
};

struct YawBasinCandidate {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    double yaw = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    Eigen::Vector3d vertex = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
};

struct Pose2DEvalResult {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    Eigen::Vector2d vertex = Eigen::Vector2d::Zero();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    double yaw = 0.0;
};

struct YawBasinCandidate2D {
    bool valid = false;
    double score = -std::numeric_limits<double>::infinity();
    double yaw = 0.0;
    Eigen::Vector2d vertex = Eigen::Vector2d::Zero();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
};

} // namespace

void PlannerManager::initPlannerModule(ros::NodeHandle &nh) {
    node_ = nh;
    gap_candidates_open_.clear();
    gap_candidates_limited_.clear();
    gap_candidates_free_.clear();

    if(env_type_){
        ROS_INFO("3D environment, using velodyne pointcloud");
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &PlannerManager::velodyneCallback, this);
        node_.param<int>("trajectory/num_of_yaw_samples", num_of_yaw_samples_, 18);
        node_.param<int>("trajectory/num_of_roll_samples", num_of_roll_samples_, 5);
        node_.param<int>("trajectory/num_of_pitch_samples", num_of_pitch_samples_, 5);

        node_.param<double>("trajectory/upper_bound_of_roll", upper_bound_of_roll_, 0.5236);
        node_.param<double>("trajectory/lower_bound_of_roll", lower_bound_of_roll_, -0.5236);
        node_.param<double>("trajectory/upper_bound_of_pitch", upper_bound_of_pitch_, 0.5236);
        node_.param<double>("trajectory/lower_bound_of_pitch", lower_bound_of_pitch_, -0.5236);

        node_.param<int>("trajectory/top_k_yaw_basins", top_k_yaw_basins_, 3);

        node_.param<double>("trajectory/fine_yaw_half_span_deg",  fine_yaw_half_span_deg_,  20.0);
        node_.param<double>("trajectory/fine_roll_half_span_deg", fine_roll_half_span_deg_, 10.0);
        node_.param<double>("trajectory/fine_pitch_half_span_deg", fine_pitch_half_span_deg_, 10.0);
        node_.param<double>("trajectory/fine_angle_step_deg", fine_angle_step_deg_, 5.0);
        std::vector<double> robot_shape_pts;
        node_.param<std::vector<double>>("robot_shape/points", robot_shape_pts, {0.1, 0.1, 0.0,
                                                                                     0.1, -0.1, 0.0,
                                                                                    -0.1, -0.1, 0.0,
                                                                                    -0.1, 0.1, 0.0,
                                                                                     0.1, 0.1, -0.2,
                                                                                     0.1, -0.1, -0.2,
                                                                                    -0.1, -0.1, -0.2,
                                                                                    -0.1, 0.1, -0.2});
        // convert to vec_Eigen format
        for (size_t i = 0; i < robot_shape_pts.size(); i += 3) {
            robot_shape_points_.push_back(Eigen::Vector3d(robot_shape_pts[i], robot_shape_pts[i+1], robot_shape_pts[i+2]));
            robot_shape_points_d_.push_back(Eigen::Vector3d(robot_shape_pts[i], robot_shape_pts[i+1], robot_shape_pts[i+2]));
        }
        Sphere3D robot_sphere;
        std::vector<Eigen::Vector3d> robot_shape_pts_eigen;
        for (const auto &pt : robot_shape_points_){
            robot_shape_pts_eigen.push_back(pt);
        }
        robot_sphere = minimumEnclosingSphere3D(robot_shape_pts_eigen);
        robot_ellipsoid_ = Ellipsoid3D(Eigen::Matrix3d::Identity() * robot_sphere.radius, robot_sphere.center);
    }
    else {
        ROS_INFO("2D environment, using 2D laser scan");
        scan2d_sub_ = node_.subscribe("/scan", 1, &PlannerManager::scan2dCallback, this);
        node_.param<int>("trajectory/num_of_yaw_samples", num_of_yaw_samples_, 18);

        node_.param<int>("trajectory/top_k_yaw_basins", top_k_yaw_basins_, 3);
        node_.param<double>("trajectory/fine_yaw_half_span_deg",  fine_yaw_half_span_deg_,  20.0);
        node_.param<double>("trajectory/fine_angle_step_deg", fine_angle_step_deg_, 5.0);
        std::vector<double> robot_shape_pts_2d;
        node_.param<std::vector<double>>("robot_shape/points_2d", robot_shape_pts_2d, {0.1, 0.1,
                                                                                         0.1, -0.1,
                                                                                        -0.1, -0.1,
                                                                                        -0.1, 0.1});
        // convert to vec_Eigen format
        for (size_t i = 0; i < robot_shape_pts_2d.size(); i += 2) {
            robot_shape_points_2d_.push_back(Eigen::Vector2d(robot_shape_pts_2d[i], robot_shape_pts_2d[i+1]));
            robot_shape_points_2d_d_.push_back(Eigen::Vector2d(robot_shape_pts_2d[i], robot_shape_pts_2d[i+1]));
        }
        Circle2D robot_sphere_2d;
        std::vector<Eigen::Vector2d> robot_shape_pts_eigen_2d;
        for (const auto &pt : robot_shape_points_2d_){
            robot_shape_pts_eigen_2d.push_back(pt);
        }
        robot_sphere_2d = minimumEnclosingCircle2D(robot_shape_pts_eigen_2d);
        robot_ellipsoid_2d_ = Ellipsoid2D(Eigen::Matrix2d::Identity() * robot_sphere_2d.radius, robot_sphere_2d.center);
    }

    candidate_gaps_sub_ = node_.subscribe("/gap_candidates", 1, &PlannerManager::candidateGapsCallback, this);

    poly_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    robot_points_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_robot_points", 1, true);
    robot_sphere_pub_ = node_.advertise<decomp_ros_msgs::EllipsoidArray>("planner_manager_sphere", 1, true);

    poly_pub_aniso_full_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_aniso_full_array", 1, true);
    poly_frtree_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_frtree_array", 1, true);
    poly_frtree2_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_frtree_array2", 1, true);
    poly_frtree3_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_frtree_array3", 1, true);

    test_cube_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_test_cube", 1, true);
    traj_vis_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_traj_vis", 1, true);
    traj_after_opt_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_traj_after_opt", 1, true);

    current_direction_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_current_direction", 1, true);
    selected_edge_poly_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("selected_edge_polyhedron", 1, true);

    traj_iter_pubs_.resize(traj_iter_pub_count_);
    for (int i = 0; i < traj_iter_pub_count_; ++i) {
        std::string topic = "/traj_vis/iter_" + std::to_string(i);
        traj_iter_pubs_[i] = node_.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
    }

    base_scan_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
	tf2_ros::Buffer tfBuffer_lidar;
	tf2_ros::TransformListener tfListener_lidar(tfBuffer_lidar);
	try{
		*(base_scan_ptr_) = tfBuffer_lidar.lookupTransform("base_link", "scan", ros::Time::now(), ros::Duration(2.0));
	}
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    const Eigen::Affine3d T_base_scan = tf2::transformToEigen(*base_scan_ptr_);
    T_base_scan_mat_ = T_base_scan.matrix().cast<float>();

    odom_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf_listener_odom_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_odom_);

    odom_timer_ = node_.createTimer(ros::Duration(0.1), &PlannerManager::odomTimerCallback, this);
    debug_timer_ = node_.createTimer(ros::Duration(0.01), &PlannerManager::debugTimerCallback, this);

    free_regions_graph_ptr_ = std::make_unique<FreeRegionsGraph>();

    gap_extractor_ptr_.reset(new GapExtractor());
    gap_extractor_ptr_->initialize(node_, env_type_);
}

void PlannerManager::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Process the Velodyne point cloud data
    if (msg->data.size() == 0)
        {
        ROS_WARN("[Planner_manager] Received empty point cloud data");
        return;
        }
    
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

    vec_Vec3f pointcloud = DecompROS::cloud_to_vec(cloud);
    vec_Vec3f pointcloud_cropped;

    // crop the point cloud
    for (const auto &pt : pointcloud) {
        if (pt[0] > -size_of_cropped_pointcloud_[0]/2 && pt[0] < size_of_cropped_pointcloud_[0]/2 &&
            pt[1] > -size_of_cropped_pointcloud_[1]/2 && pt[1] < size_of_cropped_pointcloud_[1]/2 &&
            pt[2] > -size_of_cropped_pointcloud_[2]/2 && pt[2] < size_of_cropped_pointcloud_[2]/2) {
                pointcloud_cropped.push_back(pt);
            }
    }

    // pointcloud are in scan frame, transform it into odom frame
    const Eigen::Affine3d T_odom_base = tf2::transformToEigen(*odom_base_ptr_);
    const Eigen::Matrix4f T_odom_base_mat = T_odom_base.matrix().cast<float>(); 

    vec_Vec3f pointcloud_cropped_odom_frame;
    for (const auto &pt : pointcloud_cropped) {
        Eigen::Vector4f pt_homogeneous(pt[0], pt[1], pt[2], 1.0);
        Eigen::Vector4f pt_transformed = T_odom_base_mat * T_base_scan_mat_ * pt_homogeneous;
        pointcloud_cropped_odom_frame.push_back(Eigen::Vector3d(pt_transformed[0], pt_transformed[1], pt_transformed[2]));
    }
    // add floor points at z = 0 
    // for (const auto &pt : pointcloud_cropped_odom_frame) {
    //     Eigen::Vector3d floor_pt(pt[0], pt[1], 0.0);
    //     pointcloud_cropped_odom_frame.push_back(floor_pt);
    // }
    pointcloud_cropped_odom_frame_ = pointcloud_cropped_odom_frame;
}

void PlannerManager::scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    // Process the 2D laser scan data
    if (msg->ranges.size() == 0)
        {
        ROS_WARN("[Planner_manager] Received empty laser scan data");
        return;
        }
    vec_Vec2f pointcloud_2d;
    double angle = msg->angle_min;
    for (const auto &range : msg->ranges) {
        if (range < msg->range_max && range > msg->range_min) {
            double x = range * cos(angle);
            double y = range * sin(angle);
            pointcloud_2d.push_back(Eigen::Vector2d(x, y));
        }
        angle += msg->angle_increment;
    }

    vec_Vec2f pointcloud_cropped_2d;

    // crop the point cloud
    for (const auto &pt : pointcloud_2d) {
        if (pt[0] > -size_of_cropped_pointcloud_[0]/2 && pt[0] < size_of_cropped_pointcloud_[0]/2 &&
            pt[1] > -size_of_cropped_pointcloud_[1]/2 && pt[1] < size_of_cropped_pointcloud_[1]/2) {
                pointcloud_cropped_2d.push_back(pt);
            }
    }

    // pointcloud are in scan frame, transform it into odom frame
    const Eigen::Affine3d T_odom_base = tf2::transformToEigen(*odom_base_ptr_);
    const Eigen::Matrix4f T_odom_base_mat = T_odom_base.matrix().cast<float>();

    vec_Vec2f pointcloud_cropped_2d_odom_frame;
    for (const auto &pt : pointcloud_cropped_2d) {
        Eigen::Vector4f pt_homogeneous(pt[0], pt[1], 0.0, 1.0);
        Eigen::Vector4f pt_transformed = T_odom_base_mat * T_base_scan_mat_ * pt_homogeneous;
        pointcloud_cropped_2d_odom_frame.push_back(Eigen::Vector2d(pt_transformed[0], pt_transformed[1]));
    }
    pointcloud_cropped_odom_frame_2d_ = pointcloud_cropped_2d_odom_frame;
}

void PlannerManager::odomTimerCallback(const ros::TimerEvent &event) {
    try{
        *odom_base_ptr_ = tf_buffer_odom_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void PlannerManager::getOdometryInfo(Eigen::Matrix4d &T_odom){
    T_odom(0,3) = odom_base_ptr_->transform.translation.x;
    T_odom(1,3) = odom_base_ptr_->transform.translation.y;
    T_odom(2,3) = odom_base_ptr_->transform.translation.z;
    T_odom(3,3) = 1.0;
    
    Eigen::Quaterniond q;
    q.x() = odom_base_ptr_->transform.rotation.x;
    q.y() = odom_base_ptr_->transform.rotation.y;
    q.z() = odom_base_ptr_->transform.rotation.z;
    q.w() = odom_base_ptr_->transform.rotation.w;
    T_odom.block<3,3>(0,0) = q.toRotationMatrix();
}

void PlannerManager::candidateGapsCallback(const planner_manager::GapCandidates::ConstPtr &msg) {
    gap_candidates_open_.clear();
    gap_candidates_limited_.clear();
    gap_candidates_free_.clear();

    auto emplace_candidate = [&](const planner_manager::GapCandidate &c, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &container)
    {
        Gaps gap;
        gap.type = c.type;
        gap.center_yaw = c.center_yaw;
        gap.center_elev = c.center_elev;
        gap.yaw_bias = c.yaw_bias;
        gap.elev_bias = c.elev_bias;
        gap.yaw_span = c.yaw_span;
        gap.elev_span = c.elev_span;
        gap.size = c.size;

        gap.range_mean = c.range_mean;
        gap.dir_scan_frame = decltype(gap.dir_scan_frame)(c.dir.x, c.dir.y, c.dir.z);

        container.emplace_back(std::move(gap));
    };

    for (const auto &c : msg->items){
        switch (c.type){
            case 0: emplace_candidate(c, gap_candidates_open_);     break;
            case 1: emplace_candidate(c, gap_candidates_limited_);  break;
            case 2: emplace_candidate(c, gap_candidates_free_);     break;
            default:                                                break;
        }
    }
    // ROS_INFO_STREAM_THROTTLE(1.0,
    // "[PlannerManager] Gap candidates received. open="
    // << gap_candidates_open_.size()
    // << " limited=" << gap_candidates_limited_.size()
    // << " free="    << gap_candidates_free_.size());
}

bool PlannerManager::computeSingleCorridor3DLocal(const Eigen::Vector3d& start_pos, const Gaps& gap, Polyhedron3D& out_poly)
{
    const Vec3f p1(start_pos[0], start_pos[1], start_pos[2]);
    const Vec3f p2(gap.dir_odom_frame[0], gap.dir_odom_frame[1], gap.dir_odom_frame[2]);

    // robot shape in odom frame
    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Vector3d base_pos_odom = T_odom.block<3,1>(0,3);
    const Eigen::Matrix3d base_rot_mat = T_odom.block<3,3>(0,0);

    std::vector<Vec3f> robot_shape_points_odom;
    robot_shape_points_odom.reserve(robot_shape_points_.size());
    for (const auto &pt : robot_shape_points_) {
        const Eigen::Vector3d local_pt(pt[0], pt[1], pt[2]);
        const Eigen::Vector3d rotated_pt = base_rot_mat * local_pt + base_pos_odom;
        robot_shape_points_odom.emplace_back(
            static_cast<float>(rotated_pt[0]),
            static_cast<float>(rotated_pt[1]),
            static_cast<float>(rotated_pt[2]));
    }

    const Eigen::Vector3d center_base = robot_ellipsoid_.d().cast<double>();
    const Eigen::Vector4d center_base_homo(center_base[0], center_base[1], center_base[2], 1.0);
    const Eigen::Vector4d center_odom_homo = T_odom * center_base_homo;
    const Eigen::Vector3d center_odom = center_odom_homo.head<3>();

    const double radius = robot_ellipsoid_.C()(0,0);

    LineSegment3D line_segment_aniso(p1, p2);

    // keep exactly the same bbox logic as your original version
    if (gap.type == 1 || gap.type == 3) {
        line_segment_aniso.set_local_bbox_aniso(
            Vec3f(0.5f, 0.5f, 0.5f),
            Vec3f(0.3f, 0.5f, 0.5f));
    } else {
        line_segment_aniso.set_local_bbox_aniso(
            Vec3f(0.0f, 0.5f, 0.5f),
            Vec3f(0.3f, 0.5f, 0.5f));
    }

    line_segment_aniso.set_obs_aniso(pointcloud_cropped_odom_frame_);
    line_segment_aniso.set_robot_shape_pts(robot_shape_points_odom);
    line_segment_aniso.dilate_aniso_full(
        Vec3f(center_odom[0], center_odom[1], center_odom[2]),
        radius);

    out_poly = line_segment_aniso.get_polyhedron();
    return true;
}

bool PlannerManager::computeSingleCorridor2DLocal(const Eigen::Vector3d& start_pos, const Gaps& gap, Polyhedron2D& out_poly)
{
    const Vec2f p1(start_pos[0], start_pos[1]);
    const Vec2f p2(gap.dir_odom_frame[0], gap.dir_odom_frame[1]);

    // robot shape in odom frame
    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Vector2d base_pos_odom = T_odom.block<2,1>(0,3);
    const Eigen::Matrix3d base_rot_mat = T_odom.block<3,3>(0,0);
    const Eigen::Matrix2d base_rot_2d = base_rot_mat.block<2,2>(0,0);

    std::vector<Vec2f> robot_shape_points_odom;
    robot_shape_points_odom.reserve(robot_shape_points_2d_.size());
    for (const auto &pt : robot_shape_points_2d_) {
        const Eigen::Vector2d local_pt(pt[0], pt[1]);
        const Eigen::Vector2d rotated_pt = base_rot_2d * local_pt + base_pos_odom;
        robot_shape_points_odom.emplace_back(
            static_cast<float>(rotated_pt[0]),
            static_cast<float>(rotated_pt[1]));
    }

    const Eigen::Vector2d center_base = robot_ellipsoid_2d_.d().cast<double>();
    const Eigen::Vector3d center_base_homo(center_base[0], center_base[1], 0.0);
    const Eigen::Vector3d center_odom = T_odom.block<3,3>(0,0) * center_base_homo + T_odom.block<3,1>(0,3);

    const double radius = robot_ellipsoid_2d_.C()(0,0);

    LineSegment2D line_segment_aniso(p1, p2);

    // keep exactly the same bbox logic as your original version
    if (gap.type == 1 || gap.type == 3) {
        line_segment_aniso.set_local_bbox_aniso(
            Vec2f(0.5f, 0.5f),
            Vec2f(0.3f, 0.5f));
    } else {
        line_segment_aniso.set_local_bbox_aniso(
            Vec2f(0.5f, 0.5f),
            Vec2f(0.3f, 0.5f));
    }

    line_segment_aniso.set_obs_aniso(pointcloud_cropped_odom_frame_2d_);
    line_segment_aniso.set_robot_shape_pts(robot_shape_points_odom);
    line_segment_aniso.dilate_aniso_full(
        Vec2f(center_odom[0], center_odom[1]),
        static_cast<float>(radius));

    out_poly = line_segment_aniso.get_polyhedron();
    return true;
}

void PlannerManager::decomposeAlongGapDirectionsTEST(Eigen::Vector3d &start_pos, const Gaps& gap)
{
    polys_2d_.clear();
    polys_3d_.clear();
    polys_aniso_full_.clear();
    polys_aniso_full_2d_.clear();

    if (env_type_) {
        const Vec3f p1(start_pos[0], start_pos[1], start_pos[2]);
        const Vec3f p2(gap.dir_odom_frame[0], gap.dir_odom_frame[1], gap.dir_odom_frame[2]);

        // original isotropic
        LineSegment3D line_segment(p1, p2);
        line_segment.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f));
        line_segment.set_obs(pointcloud_cropped_odom_frame_);
        auto t0 = std::chrono::high_resolution_clock::now();
        line_segment.dilate(0.1f);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
        ROS_INFO("[PlannerManager][TEST] dilate elapsed: %.3f ms", ms);

        // local anisotropic full
        Polyhedron3D aniso_poly;
        auto t2 = std::chrono::high_resolution_clock::now();
        computeSingleCorridor3DLocal(start_pos, gap, aniso_poly);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t3 - t2).count();
        ROS_INFO("[PlannerManager][TEST] dilate_aniso_full elapsed: %.3f ms", ms2);

        polys_3d_.push_back(line_segment.get_polyhedron());
        polys_aniso_full_.push_back(aniso_poly);

        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_3d_);
        decomp_ros_msgs::PolyhedronArray poly_msg_aniso_full = DecompROS::polyhedron_array_to_ros(polys_aniso_full_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_pub_.publish(poly_msg);
        poly_msg_aniso_full.header.stamp = ros::Time::now();
        poly_msg_aniso_full.header.frame_id = "odom";
        poly_pub_aniso_full_.publish(poly_msg_aniso_full);
    }
    else {
        const Vec2f p1(start_pos[0], start_pos[1]);
        const Vec2f p2(gap.dir_odom_frame[0], gap.dir_odom_frame[1]);

        LineSegment2D line_segment(p1, p2);
        line_segment.set_local_bbox(Vec2f(0.5f, 0.5f));
        line_segment.set_obs(pointcloud_cropped_odom_frame_2d_);
        auto t0 = std::chrono::high_resolution_clock::now();
        line_segment.dilate(0.1f);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
        ROS_INFO("[PlannerManager][TEST] dilate elapsed: %.3f ms", ms);

        Polyhedron2D aniso_poly;
        auto t2 = std::chrono::high_resolution_clock::now();
        computeSingleCorridor2DLocal(start_pos, gap, aniso_poly);
        auto t3 = std::chrono::high_resolution_clock::now();
        double ms2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t3 - t2).count();
        ROS_INFO("[PlannerManager][TEST] dilate_aniso_full elapsed: %.3f ms", ms2);

        polys_2d_.push_back(line_segment.get_polyhedron());
        polys_aniso_full_2d_.push_back(aniso_poly);

        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_2d_);
        decomp_ros_msgs::PolyhedronArray poly_msg_aniso_full = DecompROS::polyhedron_array_to_ros(polys_aniso_full_2d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_pub_.publish(poly_msg);
        poly_msg_aniso_full.header.stamp = ros::Time::now();
        poly_msg_aniso_full.header.frame_id = "odom";
        poly_pub_aniso_full_.publish(poly_msg_aniso_full);
    }
}

void PlannerManager::decomposeAlongGapDirections_FRTreeTEST(Eigen::Vector3d &start_pos, const Gaps& gap)
{
    polys_FRTree_2d_.clear();
    polys_FRTree_3d_.clear();

    if (!env_type_) {
        Eigen::Vector2d dir = gap.dir_odom_frame.head<2>() - start_pos.head<2>();
        double dist = dir.norm();
        if (dist < 1e-6) return;
        dir.normalize();

        Eigen::Vector2d pos1 = start_pos.head<2>() + dir * (dist * 0.05);
        Eigen::Vector2d pos2 = start_pos.head<2>() + dir * (dist * 0.45);
        Eigen::Vector2d pos3 = start_pos.head<2>() + dir * (dist * 0.40);
        Eigen::Vector2d pos4 = start_pos.head<2>() + dir * (dist * 0.70);
        Eigen::Vector2d pos5 = start_pos.head<2>() + dir * (dist * 0.65);
        Eigen::Vector2d pos6 = start_pos.head<2>() + dir * (dist * 0.90);

        const Vec2f p1(pos1[0], pos1[1]);
        const Vec2f p2(pos2[0], pos2[1]);
        const Vec2f p3(pos3[0], pos3[1]);
        const Vec2f p4(pos4[0], pos4[1]);
        const Vec2f p5(pos5[0], pos5[1]);
        const Vec2f p6(pos6[0], pos6[1]);

        LineSegment2D line_segment1(p1, p2);
        line_segment1.set_local_bbox(Vec2f(0.5f, 0.5f));
        line_segment1.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment1.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment1.get_polyhedron());

        LineSegment2D line_segment2(p3, p4);
        line_segment2.set_local_bbox(Vec2f(0.5f, 0.5f));
        line_segment2.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment2.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment2.get_polyhedron());

        LineSegment2D line_segment3(p5, p6);
        line_segment3.set_local_bbox(Vec2f(0.5f, 0.5f));
        line_segment3.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment3.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment3.get_polyhedron());

        vec_E<Polyhedron2D> single_poly1;
        single_poly1.push_back(polys_FRTree_2d_[0]);
        vec_E<Polyhedron2D> single_poly2;
        single_poly2.push_back(polys_FRTree_2d_[1]);
        vec_E<Polyhedron2D> single_poly3;
        single_poly3.push_back(polys_FRTree_2d_[2]);

        decomp_ros_msgs::PolyhedronArray poly_msg1 = DecompROS::polyhedron_array_to_ros(single_poly1);
        decomp_ros_msgs::PolyhedronArray poly_msg2 = DecompROS::polyhedron_array_to_ros(single_poly2);
        decomp_ros_msgs::PolyhedronArray poly_msg3 = DecompROS::polyhedron_array_to_ros(single_poly3);

        poly_msg1.header.stamp = ros::Time::now();
        poly_msg1.header.frame_id = "odom";
        poly_frtree_pub_.publish(poly_msg1);

        poly_msg2.header.stamp = ros::Time::now();
        poly_msg2.header.frame_id = "odom";
        poly_frtree2_pub_.publish(poly_msg2);

        poly_msg3.header.stamp = ros::Time::now();
        poly_msg3.header.frame_id = "odom";
        poly_frtree3_pub_.publish(poly_msg3);
    }
    else {
        Eigen::Vector3d dir = gap.dir_odom_frame - start_pos;
        double dist = dir.norm();
        if (dist < 1e-6) return;
        dir.normalize();

        Eigen::Vector3d pos1 = start_pos + dir * (dist * 0.05);
        Eigen::Vector3d pos2 = start_pos + dir * (dist * 0.45);
        Eigen::Vector3d pos3 = start_pos + dir * (dist * 0.40);
        Eigen::Vector3d pos4 = start_pos + dir * (dist * 0.70);
        Eigen::Vector3d pos5 = start_pos + dir * (dist * 0.65);
        Eigen::Vector3d pos6 = start_pos + dir * (dist * 0.90);

        const Vec3f p1(pos1[0], pos1[1], pos1[2]);
        const Vec3f p2(pos2[0], pos2[1], pos2[2]);
        const Vec3f p3(pos3[0], pos3[1], pos3[2]);
        const Vec3f p4(pos4[0], pos4[1], pos4[2]);
        const Vec3f p5(pos5[0], pos5[1], pos5[2]);
        const Vec3f p6(pos6[0], pos6[1], pos6[2]);

        LineSegment3D line_segment1(p1, p2);
        line_segment1.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f));
        line_segment1.set_obs(pointcloud_cropped_odom_frame_);
        line_segment1.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment1.get_polyhedron());

        LineSegment3D line_segment2(p3, p4);
        line_segment2.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f));
        line_segment2.set_obs(pointcloud_cropped_odom_frame_);
        line_segment2.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment2.get_polyhedron());

        LineSegment3D line_segment3(p5, p6);
        line_segment3.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f));
        line_segment3.set_obs(pointcloud_cropped_odom_frame_);
        line_segment3.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment3.get_polyhedron());

        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_FRTree_3d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_frtree_pub_.publish(poly_msg);
    }
}

void PlannerManager::generateNodePolyhedron(NodeId nid, const Eigen::Vector3d& start_pos) {
    auto* current_node = free_regions_graph_ptr_->getNode(nid);
    if(env_type_){
        // 3D
        SeedDecomp3D seed_decomp_3d(start_pos);
        seed_decomp_3d.set_local_bbox(Vec3f(0.8f, 0.8f, 0.8f));
        seed_decomp_3d.set_obs(pointcloud_cropped_odom_frame_);
        seed_decomp_3d.dilate(0.1);  
        Polyhedron3D node_poly = seed_decomp_3d.get_polyhedron();
        current_node->polys_ = node_poly;
        current_node->state_pos_ = start_pos;
    }
    else{
        // 2D
        SeedDecomp2D seed_decomp_2d(start_pos.head<2>());
        seed_decomp_2d.set_local_bbox(Vec2f(0.3f, 0.3f));
        seed_decomp_2d.set_obs(pointcloud_cropped_odom_frame_2d_);
        seed_decomp_2d.dilate(0.1);  
        Polyhedron2D node_poly = seed_decomp_2d.get_polyhedron();
        current_node->polys_2d_ = node_poly;
        current_node->state_pos_ = start_pos;
    }
}

void PlannerManager::pruneUntriedParentEdgesByChildPoly(NodeId parent_id, NodeId child_id, EdgeId incoming_eid)
{
    auto* parent = free_regions_graph_ptr_->getNode(parent_id);
    auto* child  = free_regions_graph_ptr_->getNode(child_id);

    if (!parent || !child) {
        ROS_WARN("[PlannerManager] pruneUntriedParentEdgesByChildPoly: invalid parent or child node.");
        return;
    }

    std::vector<EdgeId> kept_edges;
    kept_edges.reserve(parent->edge_ids_.size());

    int pruned_cnt = 0;

    for (EdgeId eid : parent->edge_ids_) {
        auto* e = free_regions_graph_ptr_->getEdge(eid);
        if (!e) continue;

        // always keep the incoming edge that actually led to the child
        if (eid == incoming_eid) {
            kept_edges.push_back(eid);
            continue;
        }

        // keep all tried edges; only prune frontier edges
        if (e->tried_) {
            kept_edges.push_back(eid);
            continue;
        }

        bool contained = false;

        if (env_type_) {
            contained = poseContainedInParentPoly3D(
                child->state_pos_,       
                child->polys_,           
                e->replan_pos_,          
                e->R_,                   
                0.0                      
            );
        }
        else {
            contained = poseContainedInParentPoly2D(
                child->state_pos_,   
                child->polys_2d_,              
                e->replan_pos_,      
                e->R_2d_,                      
                0.0
            );
        }

        if (contained) {
            pruned_cnt++;
            ROS_INFO("[PlannerManager] Prune parent edge %d at node %d because its target pose is contained in child node %d poly.",
                     eid, parent_id, child_id);
            continue; // directly drop this edge
        }

        kept_edges.push_back(eid);
    }

    parent->edge_ids_.swap(kept_edges);

    ROS_INFO("[PlannerManager] pruneUntriedParentEdgesByChildPoly: pruned %d edge(s) from parent node %d.",
             pruned_cnt, parent_id);
}

void PlannerManager::filterBackwardGaps(const Eigen::Vector3d &start_pos, NodeId current_node_id, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates){
    // first, retain all the candidates gaps
    all_candidates.clear();
    all_candidates.reserve(gap_candidates_open_.size() +
                           gap_candidates_limited_.size() +
                           gap_candidates_free_.size());
    all_candidates.insert(all_candidates.end(), gap_candidates_open_.begin(),    gap_candidates_open_.end());
    all_candidates.insert(all_candidates.end(), gap_candidates_limited_.begin(), gap_candidates_limited_.end());
    all_candidates.insert(all_candidates.end(), gap_candidates_free_.begin(),    gap_candidates_free_.end());
    if (all_candidates.empty()) {
        ROS_WARN("[PlannerManager] No gap candidates available for filtering.");
        return;
    }
    // convert the vector from scan frame to odom frame
    const Eigen::Affine3d T_odom_base = tf2::transformToEigen(*odom_base_ptr_);
    const Eigen::Matrix4f T_odom_base_mat = T_odom_base.matrix().cast<float>();

    auto to_odom_frame = [&](const Eigen::Vector3d &dir_scan_frame) -> Eigen::Vector3d {
        Eigen::Vector4f p_scan_frame(dir_scan_frame[0], dir_scan_frame[1], dir_scan_frame[2], 1.0);
        Eigen::Vector4f p_odom_frame = T_odom_base_mat * T_base_scan_mat_ * p_scan_frame;
        return Eigen::Vector3d(static_cast<double>(p_odom_frame[0]),
                               static_cast<double>(p_odom_frame[1]),
                               static_cast<double>(p_odom_frame[2]));
    };
    // compute incoming direction 
    Eigen::Vector3d incoming_dir = Eigen::Vector3d::Zero();
    bool have_incoming_dir = false;
    if (free_regions_graph_ptr_){
        auto* cur_node = free_regions_graph_ptr_->getNode(current_node_id);
        if (cur_node){
            const EdgeId incoming = cur_node->incoming_edge_id_;
            if (incoming > 0){
                auto* e_in = free_regions_graph_ptr_->getEdge(incoming);
                if (e_in && e_in->from_ >= 0){
                    auto* from_node = free_regions_graph_ptr_->getNode(e_in->from_);
                    if (from_node){
                        Eigen::Vector3d d = cur_node->state_pos_ - from_node->state_pos_;
                        const double n = d.norm();
                        if (n > 1e-6){
                            incoming_dir = d / n;
                            have_incoming_dir = true;
                        }
                    }
                }
            }
        }
    }
    // filter out the gaps that are backward facing
    const double cos_backward = std::cos(120.0 * M_PI / 180.0); // 150 degrees
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> filtered;
    filtered.reserve(all_candidates.size());
    int removed = 0;
    for (auto& gap : all_candidates){
        gap.dir_odom_frame = to_odom_frame(gap.dir_scan_frame);
        Eigen::Vector3d dir = gap.dir_odom_frame - start_pos;
        double norm = dir.norm();
        if (norm < 1e-6){
            // direction too small, keep it
            filtered.push_back(gap);
            continue;
        }
        dir /= norm;
        if (have_incoming_dir){
            const double c = dir.dot(incoming_dir);
            if (c < cos_backward){
                // this gap is backward facing, filter it out
                removed++;
                continue;
            }
        }
        filtered.push_back(gap);
    }
    all_candidates.swap(filtered);
    ROS_INFO("[PlannerManager] Filtered out %d backward-facing gaps, %lu remaining.", removed, all_candidates.size());
}

void PlannerManager::sortAllCandidatesGap(Eigen::Vector3d &goal_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates) {
    if (all_candidates.empty()) {
        ROS_WARN("[PlannerManager] No gap candidates available for sorting.");
        return;
    }

    struct Item {
        Gaps gap;
        double d;
    };

    std::vector<Item> items;
    items.reserve(all_candidates.size());
    for (const auto& g : all_candidates) {
        items.push_back({g, (goal_pos - g.dir_odom_frame).norm()});
    }

    std::sort(items.begin(), items.end(),
              [](const Item& a, const Item& b){ return a.d < b.d; });

    for (size_t i = 0; i < items.size(); ++i) {
        all_candidates[i] = std::move(items[i].gap);
    }
}

void PlannerManager::reorderCandidatesGapWithGoal(Eigen::Vector3d &goal_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates){
    if (all_candidates.empty()) return;
    gap_extractor_ptr_->checkGoalStatus(goal_pos);
    GoalStatus goal_status = gap_extractor_ptr_->getGoalStatus();
    if (goal_status == GoalStatus::BLOCKED || goal_status == GoalStatus::OUT_OF_VIEW){
        // no need to reorder
        return;
    }
ROS_INFO("[PlannerManager] Goal status: %d", static_cast<int>(goal_status));
    Eigen::Vector3d base_pose(0.0, 0.0, 0.0);
    if (odom_base_ptr_) {
        base_pose[0] = odom_base_ptr_->transform.translation.x;
        base_pose[1] = odom_base_ptr_->transform.translation.y;
        base_pose[2] = odom_base_ptr_->transform.translation.z;
    }

    Eigen::Vector3d goal_vec = goal_pos - base_pose;
    double goal_distance = goal_vec.norm();
    if (goal_distance < 1e-3) {
        // goal too close to the robot
        return;
    }
    Eigen::Vector3d u_goal = goal_vec / goal_distance;

    const double angle_margin = 5.0 * M_PI / 180.0; // 5 degrees in radians

    struct GapWithFlag {
        Gaps   gap;
        bool   contains_goal = false;  // whether this gap angular cone contains u_goal
        double theta = 0.0;           // angle between gap direction and goal direction
    };

    std::vector<GapWithFlag> temp;
    temp.reserve(all_candidates.size());

    bool any_contains_goal_open_free = false;

    for (const auto &gap : all_candidates) {
        GapWithFlag gwf;
        gwf.gap = gap;

        // Direction of this gap in odom frame (should already be set)
        Eigen::Vector3d u_gap = gap.dir_odom_frame.normalized();

        double cos_theta = u_goal.dot(u_gap);
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
        double theta = std::acos(cos_theta);  // [0, pi]
        gwf.theta = theta;

        // Use the larger of yaw_span / elev_span as an approximate cone aperture
        double half_angle = 0.5 * static_cast<double>(
            std::max(gap.yaw_span, gap.elev_span)
        );

        gwf.contains_goal = (theta <= half_angle + angle_margin);

        // Only count open / free gaps (type=0 or 2) here
        if (gwf.contains_goal && (gap.type == 0 || gap.type == 2)) {
            any_contains_goal_open_free = true;
        }

        temp.push_back(gwf);
    }
    
    // Rebuild the candidate list based on goal_status

    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> reordered;

    // CASE A: goal_status == FREE and at least one OPEN/FREE gap contains goal
    if (goal_status == GoalStatus::FREE && any_contains_goal_open_free){
        int best_index = -1;
        double best_theta = 1e9;
        for (size_t i = 0; i < temp.size(); ++i){
            const auto &gwf = temp[i];
            if (gwf.contains_goal && (gwf.gap.type == 0 || gwf.gap.type == 2)){
                if (gwf.theta < best_theta){
                    best_theta = gwf.theta;
                    best_index = static_cast<int>(i);
                }
            }
        }
        if (best_index >= 0){
            GapWithFlag &best = temp[best_index];
            best.gap.type = 3; // mark as goal gap
            best.gap.dir_odom_frame = goal_pos; // set direction to exact goal position

            reordered.reserve(all_candidates.size());
            reordered.push_back(best.gap);
            // append all other gaps in the original order
            for (size_t i = 0; i < temp.size(); ++i){
                if (static_cast<int>(i) != best_index){
                    reordered.push_back(temp[i].gap);
                }
            }
            all_candidates.swap(reordered);
            return;
        }
    }

    // CASE B: goal_status == LIMITED
    if (goal_status == GoalStatus::LIMITED ||(goal_status == GoalStatus::FREE && !any_contains_goal_open_free)){
        // build a synthetic goal gap
        Gaps goal_gap;
        goal_gap.type = 3; // goal gap
        goal_gap.dir_scan_frame = Eigen::Vector3d(0.0, 0.0, 0.0); // not used
        goal_gap.dir_odom_frame = goal_pos;
        // Spherical angles of u_goal
        double yaw = std::atan2(u_goal.y(), u_goal.x());
        double elev = std::atan2(u_goal.z(), std::sqrt(u_goal.x()*u_goal.x() + u_goal.y()*u_goal.y()));

        goal_gap.center_yaw  = static_cast<float>(yaw);
        goal_gap.center_elev = static_cast<float>(elev);
        goal_gap.range_mean  = static_cast<float>(goal_distance);
        goal_gap.yaw_span  = 0.0f;
        goal_gap.elev_span = 0.0f;
        goal_gap.size      = 1;

        reordered.clear();
        reordered.reserve(all_candidates.size() + 1);
        reordered.push_back(goal_gap);

        for (const auto &gwf : temp){
            reordered.push_back(gwf.gap);
        }
        all_candidates.swap(reordered);
        return;
    }
    ROS_INFO("[PlannerManager] goal stauts not recognized for reordering gaps.");
    return;
}

double PlannerManager::supportValueVertices(const Eigen::Vector3d &norm, const std::vector<Eigen::Vector3d> &vertices, const Eigen::Matrix3d& R){
    double h = -std::numeric_limits<double>::infinity();
    const Eigen::Vector3d n_local = R.transpose() * norm; 
    for (const auto &v : vertices){
        double val = n_local.dot(v);
        if (val > h) h = val;
    }
    return h;
}

double PlannerManager::supportValueVertices(const Eigen::Vector2d &norm, const std::vector<Eigen::Vector2d> &vertices, const Eigen::Matrix2d& R){
    double h = -std::numeric_limits<double>::infinity();
    const Eigen::Vector2d n_local = R.transpose() * norm;
    for (const auto &v : vertices){
        double val = n_local.dot(v);
        if (val > h) h = val;
    }
    return h;
}

bool PlannerManager::getTargetPose3D(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_point, const Polyhedron3D &corridor_poly, Eigen::Vector3d &out_replan_pos, Eigen::Matrix3d& out_R)
{
    vec_E<Hyperplane3D> hyperplanes = corridor_poly.hyperplanes();
    LinearConstraint3D lc(start_pos, hyperplanes);
    const Eigen::VectorXd b = lc.b();

    Eigen::Vector3d dir = goal_point - start_pos;
    const double dir_norm = dir.norm();
    if (dir_norm < 1e-8) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }
    dir /= dir_norm;

    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    const double robot_roll  = std::atan2(base_rot_mat(2,1), base_rot_mat(2,2));
    const double robot_pitch = std::asin(-base_rot_mat(2,0));
    const double robot_yaw   = std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));

    // coarse samples
    const std::vector<double> yaw_vals =
        buildUniformSamplesInclusive(-M_PI, M_PI, num_of_yaw_samples_);
    const std::vector<double> roll_vals =
        buildUniformSamplesInclusive(lower_bound_of_roll_, upper_bound_of_roll_, num_of_roll_samples_);
    const std::vector<double> pitch_vals =
        buildUniformSamplesInclusive(lower_bound_of_pitch_, upper_bound_of_pitch_, num_of_pitch_samples_);

    const int m = lc.A().rows();
    std::vector<Eigen::Vector3d> arows(m);
    std::vector<double> shrink(m);
    const double clearance = 0.02;
    for (int l = 0; l < m; ++l) {
        arows[l] = lc.A().row(l).transpose();
        shrink[l] = clearance * arows[l].norm();
    }

    const std::vector<TripleCache> triple_cache = buildTripleCache3D(arows);

    auto evaluate_pose = [&](double yaw, double roll, double pitch) -> Pose3DEvalResult {
        Pose3DEvalResult res;
        Eigen::Matrix3d R = rpyToRotation(yaw, roll, pitch);

        Eigen::VectorXd b_prime(m);
        for (int l = 0; l < m; ++l) {
            const double hl = supportValueVertices(arows[l], robot_shape_points_d_, R);
            b_prime[l] = b[l] - hl - shrink[l];
        }

        Eigen::Vector3d best_vertex = start_pos;
        const double value =
            solveLPByEnumeratingVertices3DWithCache(triple_cache, arows, b_prime, dir, best_vertex);

        if (!std::isfinite(value)) {
            return res;
        }

        const Eigen::Vector3d x_axis_world = R.col(0);
        const double align = x_axis_world.dot(dir);

        const double yaw_err   = std::abs(wrapAngleRad(yaw   - robot_yaw));
        const double roll_err  = std::abs(roll  - robot_roll);
        const double pitch_err = std::abs(pitch - robot_pitch);
        const double angle_diff = yaw_err + roll_err + pitch_err;

        const double w_align = 2.0;
        const double w_angle_diff = 0.1;
        const double score = value + align * w_align - angle_diff * w_angle_diff;

        res.valid = true;
        res.score = score;
        res.vertex = best_vertex;
        res.R = R;
        res.yaw = yaw;
        res.roll = roll;
        res.pitch = pitch;
        return res;
    };

    // ---------------- coarse: compress by yaw basin ----------------
    std::vector<YawBasinCandidate> yaw_best(yaw_vals.size());

    for (size_t iy = 0; iy < yaw_vals.size(); ++iy) {
        const double yaw = yaw_vals[iy];
        YawBasinCandidate basin_best;

        for (double roll : roll_vals) {
            for (double pitch : pitch_vals) {
                Pose3DEvalResult cur = evaluate_pose(yaw, roll, pitch);
                if (!cur.valid) continue;

                if (!basin_best.valid || cur.score > basin_best.score) {
                    basin_best.valid = true;
                    basin_best.score = cur.score;
                    basin_best.yaw = yaw;
                    basin_best.roll = roll;
                    basin_best.pitch = pitch;
                    basin_best.vertex = cur.vertex;
                    basin_best.R = cur.R;
                }
            }
        }
        yaw_best[iy] = basin_best;
    }

    std::vector<YawBasinCandidate> coarse_candidates;
    coarse_candidates.reserve(yaw_best.size());
    for (const auto& c : yaw_best) {
        if (c.valid) coarse_candidates.push_back(c);
    }

    if (coarse_candidates.empty()) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }

    std::sort(coarse_candidates.begin(), coarse_candidates.end(),
              [](const YawBasinCandidate& a, const YawBasinCandidate& b) {
                  return a.score > b.score;
              });

    const int K = std::min<int>(top_k_yaw_basins_, coarse_candidates.size());

    // ---------------- fine search around top-K yaw basins ----------------
    Pose3DEvalResult global_best;

    for (int k = 0; k < K; ++k) {
        const auto& seed = coarse_candidates[k];

        const std::vector<double> fine_yaws =
            buildCenteredSamplesDeg(seed.yaw,
                                    fine_yaw_half_span_deg_,
                                    fine_angle_step_deg_,
                                    -M_PI, M_PI, true);

        const std::vector<double> fine_rolls =
            buildCenteredSamplesDeg(seed.roll,
                                    fine_roll_half_span_deg_,
                                    fine_angle_step_deg_,
                                    lower_bound_of_roll_,
                                    upper_bound_of_roll_,
                                    false);

        const std::vector<double> fine_pitchs =
            buildCenteredSamplesDeg(seed.pitch,
                                    fine_pitch_half_span_deg_,
                                    fine_angle_step_deg_,
                                    lower_bound_of_pitch_,
                                    upper_bound_of_pitch_,
                                    false);

        for (double yaw : fine_yaws) {
            for (double roll : fine_rolls) {
                for (double pitch : fine_pitchs) {
                    Pose3DEvalResult cur = evaluate_pose(yaw, roll, pitch);
                    if (!cur.valid) continue;

                    if (!global_best.valid || cur.score > global_best.score) {
                        global_best = cur;
                    }
                }
            }
        }
    }

    // fallback: if fine failed, use best coarse
    if (!global_best.valid) {
        const auto& c = coarse_candidates.front();
        out_replan_pos = c.vertex;
        out_R = c.R;
        return true;
    }

    out_replan_pos = global_best.vertex;
    out_R = global_best.R;
    return true;
}

bool PlannerManager::getTargetPose2D(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_point, const Polyhedron2D &corridor_poly, Eigen::Vector3d &out_replan_pos, Eigen::Matrix2d& out_R)
{
    vec_E<Hyperplane2D> hyperplanes = corridor_poly.hyperplanes();
    LinearConstraint2D lc(start_pos.head<2>(), hyperplanes);
    const int m = lc.A().rows();
    const Eigen::MatrixXd A = lc.A();
    const Eigen::VectorXd b = lc.b();

    Eigen::Vector2d dir = goal_point.head<2>() - start_pos.head<2>();
    const double dir_norm = dir.norm();
    if (dir_norm < 1e-8) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }
    dir /= dir_norm;

    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    const double robot_yaw = std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));

    const std::vector<double> coarse_yaws =
        buildUniformSamplesInclusive(-M_PI, M_PI, num_of_yaw_samples_);

    auto evaluate_pose = [&](double yaw) -> Pose2DEvalResult {
        Pose2DEvalResult res;

        Eigen::Matrix2d R;
        R << std::cos(yaw), -std::sin(yaw),
             std::sin(yaw),  std::cos(yaw);

        Eigen::VectorXd b_prime(m);
        for (int j = 0; j < m; ++j) {
            Eigen::Vector2d aj = A.row(j).transpose();
            const double hj = supportValueVertices(aj, robot_shape_points_2d_d_, R);
            const double shrink = 0.02 * aj.norm();
            b_prime[j] = b[j] - hj - shrink;
        }

        Eigen::Vector2d best_vertex;
        const double value = solveLPByEnumeratingVertices2D(A, b_prime, dir, best_vertex);
        if (!std::isfinite(value)) {
            return res;
        }

        const Eigen::Vector2d x_axis_world = R.col(0);
        const double align = x_axis_world.dot(dir);

        const double yaw_err = std::abs(wrapAngleRad(yaw - robot_yaw));

        const double w_align = 1.0;
        const double w_angle_diff = 0.5;
        const double score = value + align * w_align - yaw_err * w_angle_diff;

        res.valid = true;
        res.score = score;
        res.vertex = best_vertex;
        res.R = R;
        res.yaw = yaw;
        return res;
    };

    // coarse top-K yaw
    std::vector<YawBasinCandidate2D> coarse_candidates;
    coarse_candidates.reserve(coarse_yaws.size());

    for (double yaw : coarse_yaws) {
        Pose2DEvalResult cur = evaluate_pose(yaw);
        if (!cur.valid) continue;

        YawBasinCandidate2D c;
        c.valid = true;
        c.score = cur.score;
        c.yaw = cur.yaw;
        c.vertex = cur.vertex;
        c.R = cur.R;
        coarse_candidates.push_back(c);
    }

    if (coarse_candidates.empty()) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }

    std::sort(coarse_candidates.begin(), coarse_candidates.end(),
              [](const YawBasinCandidate2D& a, const YawBasinCandidate2D& b) {
                  return a.score > b.score;
              });

    const int K = std::min<int>(top_k_yaw_basins_, coarse_candidates.size());

    Pose2DEvalResult global_best;

    for (int k = 0; k < K; ++k) {
        const auto& seed = coarse_candidates[k];

        const std::vector<double> fine_yaws =
            buildCenteredSamplesDeg(seed.yaw,
                                    fine_yaw_half_span_deg_,
                                    fine_angle_step_deg_,
                                    -M_PI, M_PI, true);

        for (double yaw : fine_yaws) {
            Pose2DEvalResult cur = evaluate_pose(yaw);
            if (!cur.valid) continue;

            if (!global_best.valid || cur.score > global_best.score) {
                global_best = cur;
            }
        }
    }

    if (!global_best.valid) {
        const auto& c = coarse_candidates.front();
        out_replan_pos.head<2>() = c.vertex;
        out_replan_pos.z() = 0.0;
        out_R = c.R;
        return true;
    }

    out_replan_pos.head<2>() = global_best.vertex;
    out_replan_pos.z() = 0.0;
    out_R = global_best.R;
    return true;
}

bool PlannerManager::getGoalPose3D(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_point, const Polyhedron3D &corridor_poly, Eigen::Vector3d &out_replan_pos, Eigen::Matrix3d& out_R)
{
    vec_E<Hyperplane3D> hyperplanes = corridor_poly.hyperplanes();
    LinearConstraint3D lc(start_pos, hyperplanes);
    const Eigen::VectorXd b = lc.b();

    Eigen::Vector3d dir = goal_point - start_pos;
    const double dir_norm = dir.norm();
    if (dir_norm < 1e-8) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }
    dir /= dir_norm;

    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    const double robot_roll  = std::atan2(base_rot_mat(2,1), base_rot_mat(2,2));
    const double robot_pitch = std::asin(-base_rot_mat(2,0));
    const double robot_yaw   = std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));

    const std::vector<double> yaw_vals =
        buildUniformSamplesInclusive(-M_PI, M_PI, num_of_yaw_samples_);
    const std::vector<double> roll_vals =
        buildUniformSamplesInclusive(lower_bound_of_roll_, upper_bound_of_roll_, num_of_roll_samples_);
    const std::vector<double> pitch_vals =
        buildUniformSamplesInclusive(lower_bound_of_pitch_, upper_bound_of_pitch_, num_of_pitch_samples_);

    const int m = lc.A().rows();
    std::vector<Eigen::Vector3d> arows(m);
    std::vector<double> shrink(m);
    const double clearance = 0.02;
    for (int l = 0; l < m; ++l) {
        arows[l] = lc.A().row(l).transpose();
        shrink[l] = clearance * arows[l].norm();
    }

    const std::vector<TripleCache> triple_cache = buildTripleCache3D(arows);

    auto evaluate_pose = [&](double yaw, double roll, double pitch) -> Pose3DEvalResult {
        Pose3DEvalResult res;
        Eigen::Matrix3d R = rpyToRotation(yaw, roll, pitch);

        Eigen::VectorXd b_prime(m);
        for (int l = 0; l < m; ++l) {
            const double hl = supportValueVertices(arows[l], robot_shape_points_d_, R);
            b_prime[l] = b[l] - hl - shrink[l];
        }

        Eigen::Vector3d best_vertex = start_pos;
        bool goal_feasible = true;
        for (int l = 0; l < m; ++l) {
            if (arows[l].dot(goal_point) > b_prime[l] + 1e-6) {
                goal_feasible = false;
                break;
            }
        }

        if (goal_feasible) {
            best_vertex = goal_point;
        } else {
            const double value =
                solveLPByEnumeratingVertices3DWithCache(triple_cache, arows, b_prime, dir, best_vertex);
            if (!std::isfinite(value)) {
                return res;
            }
        }

        const double dist_to_goal = (goal_point - best_vertex).norm();
        const Eigen::Vector3d x_axis_world = R.col(0);
        const double align = x_axis_world.dot(dir);

        const double yaw_err   = std::abs(wrapAngleRad(yaw   - robot_yaw));
        const double roll_err  = std::abs(roll  - robot_roll);
        const double pitch_err = std::abs(pitch - robot_pitch);
        const double angle_diff = yaw_err + roll_err + pitch_err;

        const double w_align = 2.0;
        const double w_angle_diff = 0.1;
        const double score = -dist_to_goal + align * w_align - angle_diff * w_angle_diff;

        res.valid = true;
        res.score = score;
        res.vertex = best_vertex;
        res.R = R;
        res.yaw = yaw;
        res.roll = roll;
        res.pitch = pitch;
        return res;
    };

    // ---------------- coarse: compress by yaw basin ----------------
    std::vector<YawBasinCandidate> yaw_best(yaw_vals.size());

    for (size_t iy = 0; iy < yaw_vals.size(); ++iy) {
        const double yaw = yaw_vals[iy];
        YawBasinCandidate basin_best;

        for (double roll : roll_vals) {
            for (double pitch : pitch_vals) {
                Pose3DEvalResult cur = evaluate_pose(yaw, roll, pitch);
                if (!cur.valid) continue;

                if (!basin_best.valid || cur.score > basin_best.score) {
                    basin_best.valid = true;
                    basin_best.score = cur.score;
                    basin_best.yaw = yaw;
                    basin_best.roll = roll;
                    basin_best.pitch = pitch;
                    basin_best.vertex = cur.vertex;
                    basin_best.R = cur.R;
                }
            }
        }
        yaw_best[iy] = basin_best;
    }

    std::vector<YawBasinCandidate> coarse_candidates;
    coarse_candidates.reserve(yaw_best.size());
    for (const auto& c : yaw_best) {
        if (c.valid) coarse_candidates.push_back(c);
    }

    if (coarse_candidates.empty()) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }

    std::sort(coarse_candidates.begin(), coarse_candidates.end(),
              [](const YawBasinCandidate& a, const YawBasinCandidate& b) {
                  return a.score > b.score;
              });

    const int K = std::min<int>(top_k_yaw_basins_, coarse_candidates.size());

    Pose3DEvalResult global_best;

    for (int k = 0; k < K; ++k) {
        const auto& seed = coarse_candidates[k];

        const std::vector<double> fine_yaws =
            buildCenteredSamplesDeg(seed.yaw,
                                    fine_yaw_half_span_deg_,
                                    fine_angle_step_deg_,
                                    -M_PI, M_PI, true);

        const std::vector<double> fine_rolls =
            buildCenteredSamplesDeg(seed.roll,
                                    fine_roll_half_span_deg_,
                                    fine_angle_step_deg_,
                                    lower_bound_of_roll_,
                                    upper_bound_of_roll_,
                                    false);

        const std::vector<double> fine_pitchs =
            buildCenteredSamplesDeg(seed.pitch,
                                    fine_pitch_half_span_deg_,
                                    fine_angle_step_deg_,
                                    lower_bound_of_pitch_,
                                    upper_bound_of_pitch_,
                                    false);

        for (double yaw : fine_yaws) {
            for (double roll : fine_rolls) {
                for (double pitch : fine_pitchs) {
                    Pose3DEvalResult cur = evaluate_pose(yaw, roll, pitch);
                    if (!cur.valid) continue;

                    if (!global_best.valid || cur.score > global_best.score) {
                        global_best = cur;
                    }
                }
            }
        }
    }

    if (!global_best.valid) {
        const auto& c = coarse_candidates.front();
        out_replan_pos = c.vertex;
        out_R = c.R;
        return true;
    }

    out_replan_pos = global_best.vertex;
    out_R = global_best.R;
    return true;
}

bool PlannerManager::getGoalPose2D(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_point, const Polyhedron2D &corridor_poly, Eigen::Vector3d &out_replan_pos, Eigen::Matrix2d& out_R)
{
    vec_E<Hyperplane2D> hyperplanes = corridor_poly.hyperplanes();
    LinearConstraint2D lc(start_pos.head<2>(), hyperplanes);
    const int m = lc.A().rows();
    const Eigen::MatrixXd A = lc.A();
    const Eigen::VectorXd b = lc.b();

    Eigen::Vector2d dir = goal_point.head<2>() - start_pos.head<2>();
    const double dir_norm = dir.norm();
    if (dir_norm < 1e-8) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }
    dir /= dir_norm;

    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    const double robot_yaw = std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));

    const std::vector<double> coarse_yaws =
        buildUniformSamplesInclusive(-M_PI, M_PI, num_of_yaw_samples_);

    auto evaluate_pose = [&](double yaw) -> Pose2DEvalResult {
        Pose2DEvalResult res;

        Eigen::Matrix2d R;
        R << std::cos(yaw), -std::sin(yaw),
             std::sin(yaw),  std::cos(yaw);

        Eigen::VectorXd b_prime(m);
        for (int j = 0; j < m; ++j) {
            Eigen::Vector2d aj = A.row(j).transpose();
            const double hj = supportValueVertices(aj, robot_shape_points_2d_d_, R);
            const double shrink = 0.02 * aj.norm();
            b_prime[j] = b[j] - hj - shrink;
        }

        Eigen::Vector2d best_vertex = start_pos.head<2>();
        bool goal_feasible = true;
        for (int l = 0; l < m; ++l) {
            if (A.row(l).dot(goal_point.head<2>()) > b_prime[l] + 1e-6) {
                goal_feasible = false;
                break;
            }
        }

        if (goal_feasible) {
            best_vertex = goal_point.head<2>();
        } else {
            const double value = solveLPByEnumeratingVertices2D(A, b_prime, dir, best_vertex);
            if (!std::isfinite(value)) {
                return res;
            }
        }

        const double dist_to_goal = (goal_point.head<2>() - best_vertex).norm();
        const Eigen::Vector2d x_axis_world = R.col(0);
        const double align = x_axis_world.dot(dir);

        const double yaw_err = std::abs(wrapAngleRad(yaw - robot_yaw));

        const double w_align = 1.0;
        const double w_angle_diff = 0.5;
        const double score = -dist_to_goal + align * w_align - yaw_err * w_angle_diff;

        res.valid = true;
        res.score = score;
        res.vertex = best_vertex;
        res.R = R;
        res.yaw = yaw;
        return res;
    };

    // coarse top-K yaw
    std::vector<YawBasinCandidate2D> coarse_candidates;
    coarse_candidates.reserve(coarse_yaws.size());

    for (double yaw : coarse_yaws) {
        Pose2DEvalResult cur = evaluate_pose(yaw);
        if (!cur.valid) continue;

        YawBasinCandidate2D c;
        c.valid = true;
        c.score = cur.score;
        c.yaw = cur.yaw;
        c.vertex = cur.vertex;
        c.R = cur.R;
        coarse_candidates.push_back(c);
    }

    if (coarse_candidates.empty()) {
        out_replan_pos = start_pos;
        out_R.setIdentity();
        return false;
    }

    std::sort(coarse_candidates.begin(), coarse_candidates.end(),
              [](const YawBasinCandidate2D& a, const YawBasinCandidate2D& b) {
                  return a.score > b.score;
              });

    const int K = std::min<int>(top_k_yaw_basins_, coarse_candidates.size());

    Pose2DEvalResult global_best;

    for (int k = 0; k < K; ++k) {
        const auto& seed = coarse_candidates[k];

        const std::vector<double> fine_yaws =
            buildCenteredSamplesDeg(seed.yaw,
                                    fine_yaw_half_span_deg_,
                                    fine_angle_step_deg_,
                                    -M_PI, M_PI, true);

        for (double yaw : fine_yaws) {
            Pose2DEvalResult cur = evaluate_pose(yaw);
            if (!cur.valid) continue;

            if (!global_best.valid || cur.score > global_best.score) {
                global_best = cur;
            }
        }
    }

    if (!global_best.valid) {
        const auto& c = coarse_candidates.front();
        out_replan_pos.head<2>() = c.vertex;
        out_replan_pos.z() = 0.0;
        out_R = c.R;
        return true;
    }

    out_replan_pos.head<2>() = global_best.vertex;
    out_replan_pos.z() = 0.0;
    out_R = global_best.R;
    return true;
} 

bool PlannerManager::poseContainedInParentPoly3D(const Eigen::Vector3d &parent_pos, const Polyhedron3D &parent_poly, const Eigen::Vector3d &p, const Eigen::Matrix3d &R, double margin){
    vec_E<Hyperplane3D> hps = parent_poly.hyperplanes();
    LinearConstraint3D lc(parent_pos, hps);
    const Eigen::MatrixXd A = lc.A();
    const Eigen::VectorXd b = lc.b();

    for (int i = 0; i < A.rows(); ++i) {
        const Eigen::Vector3d a = A.row(i).transpose();
        const double sup = supportValueVertices(a, robot_shape_points_d_, R);
        const double lhs = a.dot(p) + sup;
        if (lhs > b[i] - margin) return false;
    }
    return true;   
}
    
bool PlannerManager::poseContainedInParentPoly2D(const Eigen::Vector3d &parent_pos, const Polyhedron2D &parent_poly, const Eigen::Vector3d &p, const Eigen::Matrix2d &R, double margin){
    vec_E<Hyperplane2D> hps = parent_poly.hyperplanes();
    LinearConstraint2D lc(parent_pos.head<2>(), hps);
    const Eigen::MatrixXd A = lc.A();
    const Eigen::VectorXd b = lc.b();

    for (int i = 0; i < A.rows(); ++i) {
        const Eigen::Vector2d a = A.row(i).transpose();
        const double sup = supportValueVertices(a, robot_shape_points_2d_d_, R);
        const double lhs = a.dot(p.head<2>()) + sup;
        if (lhs > b[i] - margin) return false;
    }
    return true;    
}

std::vector<TripleCache> PlannerManager::buildTripleCache3D(
    const std::vector<Eigen::Vector3d>& Arows) 
{
    const int m = static_cast<int>(Arows.size());
    const double det_tol = 1e-10;

    std::vector<TripleCache> cache;
    cache.reserve(m * (m - 1) * (m - 2) / 6); // upper bound

    for (int i = 0; i < m; ++i) {
        const Eigen::Vector3d& a1 = Arows[i];
        for (int j = i + 1; j < m; ++j) {
            const Eigen::Vector3d& a2 = Arows[j];
            for (int k = j + 1; k < m; ++k) {
                const Eigen::Vector3d& a3 = Arows[k];

                const Eigen::Vector3d c23 = a2.cross(a3);
                const double det = a1.dot(c23);
                if (std::abs(det) < det_tol) continue;

                TripleCache tc;
                tc.i = i;
                tc.j = j;
                tc.k = k;
                tc.c23 = c23;
                tc.c31 = a3.cross(a1);
                tc.c12 = a1.cross(a2);
                tc.inv_det = 1.0 / det;

                cache.push_back(tc);
            }
        }
    }

    return cache;
}

double PlannerManager::solveLPByEnumeratingVertices3DWithCache(
    const std::vector<TripleCache>& cache,
    const std::vector<Eigen::Vector3d>& Arows,
    const Eigen::VectorXd& bprime,
    const Eigen::Vector3d& dir,
    Eigen::Vector3d& best_vertex)
{
    const int m = static_cast<int>(Arows.size());
    const double feas_tol = 1e-6;
    double best_value = -std::numeric_limits<double>::infinity();

    for (const auto& tc : cache) {
        const int i = tc.i;
        const int j = tc.j;
        const int k = tc.k;

        const Eigen::Vector3d vertex =
            ( bprime[i] * tc.c23
            + bprime[j] * tc.c31
            + bprime[k] * tc.c12 ) * tc.inv_det;

        bool feasible = true;
        for (int l = 0; l < m; ++l) {
            if (Arows[l].dot(vertex) > bprime[l] + feas_tol) {
                feasible = false;
                break;
            }
        }
        if (!feasible) continue;

        const double dist = dir.dot(vertex);
        if (dist > best_value) {
            best_value = dist;
            best_vertex = vertex;
        }
    }

    return best_value;
}
  
double PlannerManager::solveLPByEnumeratingVertices2D(const Eigen::MatrixXd &A, const Eigen::VectorXd &bprime, const Eigen::Vector2d &dir, Eigen::Vector2d &best_vertex){
    const int m = A.rows();
    const double det_tol = 1e-10;
    const double feas_tol = 1e-6;
    double best_value = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < m; ++i){
        for (int j = i + 1; j < m; ++j){
            Eigen::Matrix2d M;
            M.row(0) = A.row(i);
            M.row(1) = A.row(j);
            double a11 = M(0,0), a12 = M(0,1);
            double a21 = M(1,0), a22 = M(1,1);
            double det = a11*a22 - a12*a21;
            if (std::abs(det) < det_tol) continue; // skip near-singular
            Eigen::Vector2d rhs(bprime[i], bprime[j]);
            // Eigen::Vector2d vertex = M.inverse() * rhs;
            Eigen::Vector2d vertex;
            vertex.x() = ( rhs[0]*a22 - a12*rhs[1]) / det;
            vertex.y() = ( a11*rhs[1] - rhs[0]*a21) / det;
            // check feasibility
            bool feasible = true;
            for (int k = 0; k < m; ++k){
                double val = A.row(k).dot(vertex);
                if (val > bprime[k] + feas_tol){
                    feasible = false;
                    break;
                }
            }
            if (!feasible){
                continue;
            }
            double dist = dir.dot(vertex);
            if (dist > best_value){
                best_value = dist;
                best_vertex = vertex;
            }
        }
    }
    return best_value;
}

EdgeId PlannerManager::selectBestEdgeAtNode(NodeId nid){
    auto* node = free_regions_graph_ptr_->getNode(nid);
    if (!node || node->edge_ids_.empty()) return -1;

    for (EdgeId eid : node->edge_ids_) {
        auto* edge = free_regions_graph_ptr_->getEdge(eid);
        if (!edge) continue;
        if (edge->tried_) continue;
        return eid;
    }
    return -1;
}

EdgeId PlannerManager::getSubgoalEdgeId(NodeId current_id) const {
    auto* current_node = free_regions_graph_ptr_->getNode(current_id);
    if (!current_node || current_node->edge_ids_.empty()) return -1;
    return current_node->edge_ids_[0];
}

bool PlannerManager::isFrontierNode(NodeId nid)
{
    auto* node = free_regions_graph_ptr_->getNode(nid);
    if (!node) return false;
    if (node->deadend_) return false;
    for (EdgeId eid : node->edge_ids_) {
        auto* edge = free_regions_graph_ptr_->getEdge(eid);
        if (!edge) continue;
        if (!edge->tried_) {
            return true;
        }
    }
    return false;
}

bool PlannerManager::isTraversableEdge(EdgeId eid)
{
    auto* edge = free_regions_graph_ptr_->getEdge(eid);
    if (!edge) return false;
    if (!edge->tried_) return false;
    if (edge->from_ < 0 || edge->to_ < 0) return false;
    return true;
}

NodeId PlannerManager::otherEndpoint(EdgeId eid, NodeId u)
{
    auto* edge = free_regions_graph_ptr_->getEdge(eid);
    if (!edge) return -1;

    if (edge->from_ == u) return edge->to_;
    if (edge->to_ == u) return edge->from_;
    return -1;
}

double PlannerManager::edgeTravelCost(NodeId u, EdgeId eid)
{
    auto* node = free_regions_graph_ptr_->getNode(u);
    auto* edge = free_regions_graph_ptr_->getEdge(eid);

    if (!node || !edge) {
        return std::numeric_limits<double>::infinity();
    }

    return (node->state_pos_ - edge->replan_pos_).norm();
}

void PlannerManager::runDijkstraFrom(NodeId start_nid, std::vector<double>& dist, std::vector<NodeId>& parent_node, std::vector<EdgeId>& parent_edge){
    const int N = free_regions_graph_ptr_->numNodes();
    
    dist.assign(N, std::numeric_limits<double>::infinity());
    parent_node.assign(N, -1);
    parent_edge.assign(N, -1);

    if (start_nid < 0 || start_nid >= N){
        ROS_WARN("[PlannerManager] runDijkstraFrom: invalid start_nid");
        return;
    }

    using QueueItem = std::pair<double, NodeId>; // (dist, node_id)
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;

    dist[start_nid] = 0.0;
    pq.push({0.0, start_nid});

    while (!pq.empty()){
        const auto [cur_dist, u] = pq.top();
        pq.pop();

        if (cur_dist > dist[u])           continue;

        auto* node_u = free_regions_graph_ptr_->getNode(u);
        if (!node_u) continue;

        for (EdgeId eid : node_u->edge_ids_){
            if (!isTraversableEdge(eid)) continue;

            NodeId v = otherEndpoint(eid, u);
            if (v < 0) continue;

            const double w = edgeTravelCost(u, eid);
            if(!std::isfinite(w)) continue;

            const double new_dist = dist[u] + w;
            if (new_dist < dist[v]){
                dist[v] = new_dist;
                parent_node[v] = u;
                parent_edge[v] = eid;
                pq.push({new_dist, v});
            }
        }
    }
}

bool PlannerManager::reconstructPathToNode(NodeId start_nid, NodeId target_nid, const std::vector<NodeId>& parent_node, const std::vector<EdgeId>& parent_edge, std::vector<EdgeId>& out_path_edges){
    out_path_edges.clear();
    
    if (start_nid < 0 || target_nid < 0) return false;
    if (start_nid == target_nid) return true;

    std::vector<EdgeId> rev_edgees;
    NodeId cur = target_nid;
    while (cur != start_nid){
        if (cur < 0 || cur >= static_cast<NodeId>(parent_node.size())) return false;

        const NodeId pn = parent_node[cur];
        const EdgeId pe = parent_edge[cur];
        if (pn < 0 || pe < 0) return false;
        rev_edgees.push_back(pe);
        cur = pn;
    }
    // Reverse the path
    out_path_edges.assign(rev_edgees.rbegin(), rev_edgees.rend());
    return true;
}

bool PlannerManager::selectBestFrontierNode(NodeId current_nid, const Eigen::Vector3d &global_goal, NodeId &out_frontier_nid, std::vector<EdgeId> &out_path_edges, EdgeId &out_expand_edge_id){
    out_frontier_nid = -1;
    out_path_edges.clear();
    out_expand_edge_id = -1;

    if (!free_regions_graph_ptr_) {
        ROS_WARN("[PlannerManager] selectBestFrontierNode: graph ptr is null.");
        return false;
    }

    auto* current_node = free_regions_graph_ptr_->getNode(current_nid);
    if (!current_node) {
        ROS_WARN("[PlannerManager] selectBestFrontierNode: current node invalid.");
        return false;
    }    
    
    // run Dijkstra from current node
    std::vector<double> dist;
    std::vector<NodeId> parent_node;
    std::vector<EdgeId> parent_edge;
    runDijkstraFrom(current_nid, dist, parent_node, parent_edge);

    const int N = free_regions_graph_ptr_->numNodes();

    // scan all reachable frontier nodes and choose the globally best one
    double best_cost = std::numeric_limits<double>::infinity();
    NodeId best_nid = -1;
    EdgeId best_local_eid = -1;

    for (NodeId nid = 0; nid < N; ++nid){
        auto* node = free_regions_graph_ptr_->getNode(nid);
        if (!node) continue;
        if (node->deadend_) continue;
        if (!isFrontierNode(nid)) continue;
        if (!std::isfinite(dist[nid])) continue;

        EdgeId local_eid = selectBestEdgeAtNode(nid);
        if (local_eid < 0) continue;
        auto* edge = free_regions_graph_ptr_->getEdge(local_eid);
        if (!edge) continue;

        const double score = dist[nid] + (node->state_pos_ - edge->goal_).norm() + (edge->goal_ - global_goal).norm();
        if (score < best_cost){
            best_cost = score;
            best_nid = nid;
            best_local_eid = local_eid;
        }
    }
    if (best_nid < 0 || best_local_eid < 0){
        ROS_WARN("[PlannerManager] selectBestFrontierNode: no frontier node found");
        return false;
    }
    // reconstruct path
    std::vector<EdgeId> path_edges;
    if (!reconstructPathToNode(current_nid, best_nid, parent_node, parent_edge, path_edges)){
        ROS_WARN("[PlannerManager] selectBestFrontierNode: failed to reconstruct path");
        return false;
    }

    out_frontier_nid = best_nid;
    out_path_edges = path_edges;
    out_expand_edge_id = best_local_eid;

    ROS_INFO("[PlannerManager] selectBestFrontierNode: best frontier node = %d, "
            "path_len = %zu, expand_edge = %d, score = %.3f",
            out_frontier_nid,
            out_path_edges.size(),
            out_expand_edge_id,
            best_cost);

    return true;
}

bool PlannerManager::planTrajectoryToEdge3D(const Eigen::Vector3d &start_pos, EdgeId edge_id) {
    auto* e0 = free_regions_graph_ptr_->getEdge(edge_id);
    if (!e0) return false;

    Eigen::Vector3d start_xyz = start_pos;
    Eigen::Vector3d goal_xyz  = e0->replan_pos_;
    
    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    Eigen::Vector3d base_pos_odom(0.0, 0.0, 0.0);
    base_pos_odom = T_odom.block<3,1>(0,3);
    // get rpy in odom frame
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    double robot_roll =  std::atan2(base_rot_mat(2,1), base_rot_mat(2,2));
    double robot_pitch = std::atan2(-base_rot_mat(2,0), std::sqrt(base_rot_mat(2,1)*base_rot_mat(2,1) + base_rot_mat(2,2)*base_rot_mat(2,2)));
    double robot_yaw =   std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));
    // get goal yaw based on R
    Eigen::Matrix3d R_goal = e0->R_;
    double goal_roll =  std::atan2(R_goal(2,1), R_goal(2,2));
    double goal_pitch = std::atan2(-R_goal(2,0), std::sqrt(R_goal(2,1)*R_goal(2,1) + R_goal(2,2)*R_goal(2,2)));
    double goal_yaw =   std::atan2(R_goal(1,0), R_goal(0,0));
    BezierSE3 traj = BezierSE3::initFromEndpoints(start_xyz, Eigen::Vector3d(robot_roll, robot_pitch, robot_yaw),
                                                    goal_xyz,  Eigen::Vector3d(goal_roll, goal_pitch, goal_yaw));

    VerifyOptions opt;
    opt.eps = 1e-6;
    opt.min_dt = 5e-3;
    opt.max_nodes = 1000;
    opt.unit_normals = false;

    std::vector<Eigen::Vector3d> verts;
    // put robot_shape_points_ in verts
    for (const auto &pt : robot_shape_points_){
        verts.push_back(Eigen::Vector3d(pt[0], pt[1], pt[2]));
    }

    vec_E<Hyperplane3D> hyperplanes = e0->corridor_3d_.hyperplanes();
    LinearConstraint3D lc(start_pos, hyperplanes);
    Eigen::MatrixXd A = lc.A();
    Eigen::VectorXd b = lc.b();

    WorstViolation wv = FindWorstViolationContinuous3D(A, b, verts, traj, opt);
    if (wv.safe){
        ROS_INFO("[PlannerManager] Found a safe trajectory");
        publishTrajectoryForVisualization(traj);
    }
    else{
        // Not safe, update the traj
        ROS_INFO("Bezier verify: safe=%d, g=%g, t=%g, k=%d, i=%d",
                (int)wv.safe, wv.g, wv.t, wv.plane_k, wv.vert_i);

        publishTrajectoryForVisualization(traj, wv.t);
auto t4 = std::chrono::high_resolution_clock::now();
        for (int it=0; it<30; ++it) {
            bool ok = RepairOnce_PIQP(A,b,verts,traj,opt,wv,
                /*Kcp=*/4,
                /*topKplanes=*/5,
                /*eps_add=*/1e-6,
                /*margin=*/1e-4,
                /*delta_p=*/0.10,
                /*delta_th=*/0.20,
                /*w_p=*/1.0,
                /*w_th=*/1.0,
                /*w_slack=*/1e4);
            if (!ok) {
                publishTrajectoryForVisualization(traj, wv.t);
                ROS_WARN("repair failed");
                break;
            }
            wv = FindWorstViolationContinuous3D(A,b,verts,traj,opt);
            // ROS_INFO("[it %d] safe=%d g=%g t=%g", it, (int)wv.safe, wv.g, wv.t);
            if (wv.safe) break;
        }
auto t5 = std::chrono::high_resolution_clock::now();
double ms_opt = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t5 - t4).count();
ROS_INFO("[PlannerManager] trajectory optimization elapsed: %.3f ms", ms_opt);
    }
    e0->has_traj_ = true;
    e0->traj_is_se3_ = true;
    e0->traj3_ = traj;
    e0->tried_ = true;
    publishTrajectoryAfterOptimization(traj);
    return true;
}

bool PlannerManager::planTrajectoryToEdge2D(const Eigen::Vector3d &start_pos, EdgeId edge_id) {
    auto* e0 = free_regions_graph_ptr_->getEdge(edge_id);
    if (!e0) return false;

    Eigen::Vector3d start_xyz = start_pos;
    Eigen::Vector3d goal_xyz  = e0->replan_pos_;
    
    Eigen::Matrix4d T_odom;
    getOdometryInfo(T_odom);
    Eigen::Vector3d base_pos_odom(0.0, 0.0, 0.0);
    base_pos_odom = T_odom.block<3,1>(0,3);
    // get rpy in odom frame
    const Eigen::Matrix3d base_rot_mat = Eigen::Matrix3d(T_odom.block<3,3>(0,0));
    double robot_yaw = std::atan2(base_rot_mat(1,0), base_rot_mat(0,0));
    // get goal yaw based on R
    Eigen::Matrix2d R_goal = e0->R_2d_;
    double goal_yaw = std::atan2(R_goal(1,0), R_goal(0,0));

    BezierSE2 traj = BezierSE2::initFromEndpoints(start_xyz.head<2>(), robot_yaw,
                                                    goal_xyz.head<2>(),  goal_yaw);

    VerifyOptions opt;
    opt.eps = 1e-6;
    opt.min_dt = 5e-3;
    opt.max_nodes = 1000;
    opt.unit_normals = false;

    std::vector<Eigen::Vector2d> verts;
    // put robot_shape_points_ in verts
    for (const auto &pt : robot_shape_points_2d_){
        verts.push_back(Eigen::Vector2d(pt[0], pt[1]));
    }

    vec_E<Hyperplane2D> hyperplanes = e0->corridor_2d_.hyperplanes();
    LinearConstraint2D lc(start_pos.head<2>(), hyperplanes);
    Eigen::MatrixXd A = lc.A();
    Eigen::VectorXd b = lc.b();

    WorstViolation wv = FindWorstViolationContinuous2D(A, b, verts, traj, opt);
    if (wv.safe){
        ROS_INFO("[PlannerManager] Found a safe trajectory");
        publishTrajectoryForVisualization(traj);
    }
    else{
        // Not safe, update the traj
        ROS_INFO("Bezier verify: safe=%d, g=%g, t=%g, k=%d, i=%d",
                (int)wv.safe, wv.g, wv.t, wv.plane_k, wv.vert_i);

        publishTrajectoryForVisualization(traj, wv.t);
auto t4 = std::chrono::high_resolution_clock::now();
        for (int it=0; it<30; ++it) {
std::cout << "Iteration " << it << ": repairing trajectory..." << std::endl;
            bool ok = RepairOnce_PIQP(A,b,verts,traj,opt,wv,
                /*Kcp=*/4,
                /*topKplanes=*/5,
                /*eps_add=*/1e-6,
                /*margin=*/1e-4,
                /*delta_p=*/0.10,
                /*delta_th=*/0.10,
                /*w_p=*/1.0,
                /*w_th=*/0.1,
                /*w_slack=*/1e4);
            if (!ok) {
                publishTrajectoryForVisualization(traj, wv.t);
                ROS_WARN("repair failed");
                break;
            }
            wv = FindWorstViolationContinuous2D(A,b,verts,traj,opt);
publishTrajectoryForVisualizationIter(traj, wv.t, it);
std::cout << "Iteration " << it << ": worst violation g = " << wv.g << ", t = " << wv.t << std::endl;
std::cout << "After iteration, safe = " << wv.safe << std::endl;
            // ROS_INFO("[it %d] safe=%d g=%g t=%g", it, (int)wv.safe, wv.g, wv.t);
            if (wv.safe) break;
        }
        if (!wv.safe){
            publishTrajectoryForVisualization(traj, wv.t);
            ROS_WARN("Failed to find a safe trajectory after optimization");
        }
auto t5 = std::chrono::high_resolution_clock::now();
double ms_opt = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t5 - t4).count();
ROS_INFO("[PlannerManager] trajectory optimization elapsed: %.3f ms", ms_opt);
    }
    e0->has_traj_ = true;
    e0->traj_is_se3_ = false;
    e0->traj2_ = traj;
    e0->tried_ = true;
    publishTrajectoryAfterOptimization(traj);
    return true;
}

void PlannerManager::expandNodePrimaryOnly(Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos, NodeId current_id)
{
auto t_total_0 = std::chrono::high_resolution_clock::now();
    // 1) generate node polyhedron at current pose
    generateNodePolyhedron(current_id, start_pos);

    auto* current_node = free_regions_graph_ptr_->getNode(current_id);
    if (!current_node) return;

    // 2) prune parent's untried edges
    if (current_node->incoming_edge_id_ >= 0) {
        auto* in_edge = free_regions_graph_ptr_->getEdge(current_node->incoming_edge_id_);
        if (in_edge) {
            NodeId parent_id = in_edge->from_;
            pruneUntriedParentEdgesByChildPoly(parent_id, current_id, current_node->incoming_edge_id_);
        }
    }

    // 3) collect / filter / sort candidates
auto t_pre_0 = std::chrono::high_resolution_clock::now();

    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> all_candidates;
    filterBackwardGaps(start_pos, current_id, all_candidates);
    sortAllCandidatesGap(goal_pos, all_candidates);

    if (all_candidates.empty()) {
        ROS_WARN("[PlannerManager] No gap candidates available for planning.");
        current_node->deadend_ = true;
        return;
    }

    reorderCandidatesGapWithGoal(goal_pos, all_candidates);

    current_direction_for_visualization_[0] = all_candidates[0].dir_odom_frame[0];
    current_direction_for_visualization_[1] = all_candidates[0].dir_odom_frame[1];
    current_direction_for_visualization_[2] = env_type_ ? all_candidates[0].dir_odom_frame[2] : 0.0;
    current_pos = start_pos;

auto t_pre_1 = std::chrono::high_resolution_clock::now();
double ms_pre = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pre_1 - t_pre_0).count();
ROS_INFO("[PlannerManager] expandNodePrimaryOnly preprocess elapsed: %.3f ms", ms_pre);

    // optional test visualization: only for the first ranked gap
    // decomposeAlongGapDirectionsTEST(start_pos, all_candidates[0]);
    // decomposeAlongGapDirections_FRTreeTEST(start_pos, all_candidates[0]);

    {
        std::lock_guard<std::mutex> lk(graph_mutex_);
        current_node->edge_ids_.clear();
    }

    // 4) synchronously try candidates in order until one edge is successfully added
    bool found_primary = false;
    size_t primary_idx = 0;

    for (size_t i = 0; i < all_candidates.size(); ++i) {
        const Gaps& gap = all_candidates[i];

        auto t_decomp_0 = std::chrono::high_resolution_clock::now();

        if (env_type_) {
            Polyhedron3D corridor_poly;
            if (!computeSingleCorridor3DLocal(start_pos, gap, corridor_poly)) {
                ROS_WARN("[PlannerManager] candidate %zu corridor computation failed.", i);
                continue;
            }

auto t_decomp_1 = std::chrono::high_resolution_clock::now();
double ms_decomp = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_decomp_1 - t_decomp_0).count();

auto t_pose_0 = std::chrono::high_resolution_clock::now();

            Eigen::Vector3d replan_pos = start_pos;
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            bool ok = false;

            if (gap.type == 3) {
                ok = getGoalPose3D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            } else {
                ok = getTargetPose3D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            }

auto t_pose_1 = std::chrono::high_resolution_clock::now();
double ms_pose = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pose_1 - t_pose_0).count();

ROS_INFO("[PlannerManager] candidate %zu sync trial: decompose=%.3f ms, pose=%.3f ms, ok=%d", i, ms_decomp, ms_pose, (int)ok);
ROS_INFO("[PlannerManager] size of corridor polyhedron: num hyperplanes = %zu", corridor_poly.hyperplanes().size());

            if (!ok) {
                continue;
            }

            bool contained = false;
            {
                std::lock_guard<std::mutex> lk(graph_mutex_);
                auto* node = free_regions_graph_ptr_->getNode(current_id);
                if (!node) return;

                contained = poseContainedInParentPoly3D(start_pos, node->polys_, replan_pos, R, 0.0);
                ROS_INFO("[PlannerManager] candidate %zu contained in parent poly = %d",
                         i, (int)contained);

                if (!contained) {
                    EdgeId edge_id = free_regions_graph_ptr_->addEdge(current_id, gap.dir_odom_frame);
                    auto* e = free_regions_graph_ptr_->getEdge(edge_id);
                    if (e) {
                        e->corridor_3d_ = corridor_poly;
                        e->replan_pos_ = replan_pos;
                        e->R_ = R;
                        e->cost_ = (e->replan_pos_ - node->state_pos_).norm();

                        found_primary = true;
                        primary_idx = i;

                        ROS_INFO("[PlannerManager] candidate %zu selected as primary edge, edge_id=%d",
                                 i, edge_id);
                    }
                }
            }

            if (found_primary) break;
        }
        else {
            Polyhedron2D corridor_poly;
            if (!computeSingleCorridor2DLocal(start_pos, gap, corridor_poly)) {
                ROS_WARN("[PlannerManager] candidate %zu corridor computation failed.", i);
                continue;
            }

auto t_decomp_1 = std::chrono::high_resolution_clock::now();
double ms_decomp = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_decomp_1 - t_decomp_0).count();

auto t_pose_0 = std::chrono::high_resolution_clock::now();

            Eigen::Vector3d replan_pos = start_pos;
            Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
            bool ok = false;

            if (gap.type == 3) {
                ok = getGoalPose2D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            } else {
                ok = getTargetPose2D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            }

auto t_pose_1 = std::chrono::high_resolution_clock::now();
double ms_pose = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pose_1 - t_pose_0).count();

ROS_INFO("[PlannerManager] candidate %zu sync trial: decompose=%.3f ms, pose=%.3f ms, ok=%d",
            i, ms_decomp, ms_pose, (int)ok);

            if (!ok) {
                continue;
            }

            bool contained = false;
            {
                std::lock_guard<std::mutex> lk(graph_mutex_);
                auto* node = free_regions_graph_ptr_->getNode(current_id);
                if (!node) return;

                contained = poseContainedInParentPoly2D(start_pos, node->polys_2d_, replan_pos, R, 0.0);
                ROS_INFO("[PlannerManager] candidate %zu contained in parent poly = %d",
                         i, (int)contained);

                if (!contained) {
                    EdgeId edge_id = free_regions_graph_ptr_->addEdge(current_id, gap.dir_odom_frame);
                    auto* e = free_regions_graph_ptr_->getEdge(edge_id);
                    if (e) {
                        e->corridor_2d_ = corridor_poly;
                        e->replan_pos_ = replan_pos;
                        e->R_2d_ = R;
                        e->cost_ = (e->replan_pos_.head<2>() - node->state_pos_.head<2>()).norm();

                        found_primary = true;
                        primary_idx = i;

                        ROS_INFO("[PlannerManager] candidate %zu selected as primary edge, edge_id=%d",
                                 i, edge_id);
                    }
                }
            }

            if (found_primary) break;
        }
    }

    // 5) store only the remaining UNTRIED candidates for background expansion
auto t_pending_0 = std::chrono::high_resolution_clock::now();
    {
        std::lock_guard<std::mutex> lk(bg_job_mutex_);

        if (!shutting_down_) {
            PendingExpandJob job;
            job.node_id = current_id;
            job.start_pos = start_pos;
            job.candidates.clear();

            for (size_t i = 0; i < all_candidates.size(); ++i) {
                if (found_primary && i == primary_idx) continue;
                job.candidates.push_back(all_candidates[i]);
            }

            job.valid = !job.candidates.empty();

            if (job.valid) {
                pending_expand_jobs_.push_back(std::move(job));
                ROS_INFO("[PlannerManager] Queued background expansion job for node %d. Queue size = %zu",
                        current_id, pending_expand_jobs_.size());
            }
        }
    }
auto t_pending_1 = std::chrono::high_resolution_clock::now();
double ms_pending = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_pending_1 - t_pending_0).count();
ROS_INFO("[PlannerManager] expandNodePrimaryOnly store-pending elapsed: %.3f ms", ms_pending);

    // 6) mark deadend only based on current available edge(s)
    {
        std::lock_guard<std::mutex> lk(graph_mutex_);
        if (current_node->edge_ids_.empty() || !isFrontierNode(current_id)) {
            current_node->deadend_ = true;
        } else {
            current_node->deadend_ = false;
        }
    }

    publishTestCube();

auto t_total_1 = std::chrono::high_resolution_clock::now();
double ms_total = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t_total_1 - t_total_0).count();
ROS_INFO("[PlannerManager] expandNodePrimaryOnly total elapsed: %.3f ms", ms_total);
}

void PlannerManager::startBackgroundExpansion()
{
    if (shutting_down_) {
        ROS_WARN("[PlannerManager] Skip starting background expansion because PlannerManager is shutting down.");
        return;
    }

    bool has_job = false;
    size_t queue_size = 0;
    NodeId newest_node_id = -1;

    {
        std::lock_guard<std::mutex> lk(bg_job_mutex_);
        has_job = !pending_expand_jobs_.empty();
        queue_size = pending_expand_jobs_.size();
        if (has_job) {
            newest_node_id = pending_expand_jobs_.back().node_id;
        }
    }

    if (!has_job) return;

    if (background_expand_running_) {
        ROS_WARN("[PlannerManager] Background expansion for node %d is queued because node %d is still being processed. Queue size = %zu",
                 newest_node_id, background_running_node_id_, queue_size);
        return;
    }

    if (background_expand_thread_.joinable()) {
        background_expand_thread_.join();
    }

    background_expand_running_ = true;
    background_expand_thread_ = std::thread(&PlannerManager::backgroundExpandWorker, this);
}

void PlannerManager::backgroundExpandWorker()
{
    for (;;) {
        if (shutting_down_) {
            background_running_node_id_ = -1;
            background_expand_running_ = false;
            ROS_INFO("[PlannerManager] Background expansion worker exits: shutting down.");
            return;
        }

        PendingExpandJob job;

        {
            std::lock_guard<std::mutex> lk(bg_job_mutex_);
            if (pending_expand_jobs_.empty()) {
                background_running_node_id_ = -1;
                background_expand_running_ = false;
                ROS_INFO("[PlannerManager] Background expansion worker exits: queue is empty.");
                return;
            }

            job = std::move(pending_expand_jobs_.front());
            pending_expand_jobs_.pop_front();
            background_running_node_id_ = job.node_id;

            ROS_INFO("[PlannerManager] Background expansion dequeued node %d. Remaining queue size = %zu",
                     job.node_id, pending_expand_jobs_.size());
        }

        if (!job.valid || job.node_id < 0 || job.candidates.empty()) {
            ROS_WARN("[PlannerManager] Skip invalid background expansion job.");
            continue;
        }

        ROS_INFO("[PlannerManager] Background expansion started for node %d with %zu pending candidates.",
                 job.node_id, job.candidates.size());

        expandChildrenBackgroundParallel(job.start_pos, job.node_id, job.candidates);

        ROS_INFO("[PlannerManager] Background expansion finished for node %d. All pending candidates have been processed.",
                 job.node_id);
    }
}

void PlannerManager::expandChildrenBackgroundParallel(const Eigen::Vector3d& start_pos, NodeId current_node_id, const std::vector<Gaps, Eigen::aligned_allocator<Gaps>>& all_candidates)
{
    if (all_candidates.empty()) return;

    auto* current_node = free_regions_graph_ptr_->getNode(current_node_id);
    if (!current_node) return;

    Eigen::setNbThreads(1);
    const int N = static_cast<int>(all_candidates.size());

    // snapshot parent poly once
    Polyhedron3D parent_poly_3d;
    Polyhedron2D parent_poly_2d;
    Eigen::Vector3d parent_state_pos = current_node->state_pos_;

    if (env_type_) {
        parent_poly_3d = current_node->polys_;
    } else {
        parent_poly_2d = current_node->polys_2d_;
    }

    std::vector<BgExpandResult> results(N);

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < N; ++i) {
        const auto& gap = all_candidates[i];
        BgExpandResult local_result;
        local_result.goal = gap.dir_odom_frame;

        if (env_type_) {
            Polyhedron3D corridor_poly;
            if (!computeSingleCorridor3DLocal(start_pos, gap, corridor_poly)) {
                results[i] = local_result;
                continue;
            }

            Eigen::Vector3d replan_pos;
            Eigen::Matrix3d R;
            bool ok = false;

            if (gap.type == 3) {
                ok = getGoalPose3D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            } else {
                ok = getTargetPose3D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            }

            if (!ok) {
                results[i] = local_result;
                continue;
            }

            if (poseContainedInParentPoly3D(start_pos, parent_poly_3d, replan_pos, R, 0.0)) {
                results[i] = local_result;
                continue;
            }

            local_result.ok = true;
            local_result.replan = replan_pos;
            local_result.R3 = R;
            local_result.corridor_3d = corridor_poly;
        }
        else {
            Polyhedron2D corridor_poly;
            if (!computeSingleCorridor2DLocal(start_pos, gap, corridor_poly)) {
                results[i] = local_result;
                continue;
            }

            Eigen::Vector3d replan_pos;
            Eigen::Matrix2d R;
            bool ok = false;

            if (gap.type == 3) {
                ok = getGoalPose2D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            } else {
                ok = getTargetPose2D(start_pos, gap.dir_odom_frame, corridor_poly, replan_pos, R);
            }

            if (!ok) {
                results[i] = local_result;
                continue;
            }

            if (poseContainedInParentPoly2D(start_pos, parent_poly_2d, replan_pos, R, 0.0)) {
                results[i] = local_result;
                continue;
            }

            local_result.ok = true;
            local_result.replan = replan_pos;
            local_result.R2 = R;
            local_result.corridor_2d = corridor_poly;
        }

        results[i] = std::move(local_result);
    }

    // -------- serial merge into graph --------
    std::lock_guard<std::mutex> lk(graph_mutex_);

    auto* node = free_regions_graph_ptr_->getNode(current_node_id);
    if (!node) {
        ROS_WARN("[PlannerManager] expandChildrenBackgroundParallel: node %d invalid during merge.",
                 current_node_id);
        return;
    }

    for (int i = 0; i < N; ++i) {
        const auto& r = results[i];
        if (!r.ok) continue;

        EdgeId edge_id = free_regions_graph_ptr_->addEdge(current_node_id, r.goal);
        auto* e = free_regions_graph_ptr_->getEdge(edge_id);
        if (!e) continue;

        e->replan_pos_ = r.replan;

        if (env_type_) {
            e->R_ = r.R3;
            e->corridor_3d_ = r.corridor_3d;
            e->cost_ = (e->replan_pos_ - node->state_pos_).norm();
        } else {
            e->R_2d_ = r.R2;
            e->corridor_2d_ = r.corridor_2d;
            e->cost_ = (e->replan_pos_.head<2>() - node->state_pos_.head<2>()).norm();
        }
    }

    if (!node->edge_ids_.empty()) {
        node->deadend_ = false;
    }
}

bool PlannerManager::planGlobalBestAction(const Eigen::Vector3d &global_goal)
{
    planned_path_edges_.clear();
    planned_path_index_ = 0;
    target_frontier_node_id_ = -1;
    target_expand_edge_id_ = -1;
    current_edge_id_ = -1;
    current_edge_exec_type_ = EdgeExecType::NONE;

    NodeId frontier_nid = -1;
    std::vector<EdgeId> path_edges;
    EdgeId expand_eid = -1;

    bool ok = selectBestFrontierNode(current_node_id_,
                                     global_goal,
                                     frontier_nid,
                                     path_edges,
                                     expand_eid);
    if (!ok) {
        ROS_WARN("[PlannerManager] planGlobalBestAction: failed to find best frontier.");
        return false;
    }

    target_frontier_node_id_ = frontier_nid;
    target_expand_edge_id_ = expand_eid;
    planned_path_edges_ = path_edges;
    planned_path_index_ = 0;

    // Case A: already at the best frontier node
    if (planned_path_edges_.empty()) {
        current_edge_id_ = target_expand_edge_id_;
        current_edge_exec_type_ = EdgeExecType::EXPAND_EDGE;
        ROS_INFO("[PlannerManager] planGlobalBestAction: already at frontier node %d, execute edge %d.",
                 target_frontier_node_id_, current_edge_id_);
        return true;
    }

    // Case B: need to move along graph path first
    current_edge_id_ = planned_path_edges_[0];
    current_edge_exec_type_ = EdgeExecType::PATH_EDGE;
    ROS_INFO("[PlannerManager] planGlobalBestAction: move along graph path first. "
             "frontier node = %d, path_len = %zu, first edge = %d, final expand edge = %d",
             target_frontier_node_id_,
             planned_path_edges_.size(),
             current_edge_id_,
             target_expand_edge_id_);

    return true;
}

bool PlannerManager::prepareTrajectoryForCurrentEdge(const Eigen::Vector3d &start_pos){
    if (current_edge_id_ < 0) {
        ROS_WARN("[PlannerManager] planTrajectoryToCurrentEdge: current_edge_id_ invalid.");
        return false;
    }

    auto* edge = free_regions_graph_ptr_->getEdge(current_edge_id_);
    if (!edge) {
        ROS_WARN("[PlannerManager] planTrajectoryToCurrentEdge: current edge invalid.");
        return false;
    }

    if (env_type_) {
        return planTrajectoryToEdge3D(start_pos, current_edge_id_);
    } else {
        return planTrajectoryToEdge2D(start_pos, current_edge_id_);
    }   
}

void PlannerManager::publishRobotPoints() {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker robot_marker;
    robot_marker.header.frame_id = "base_link";
    robot_marker.header.stamp = ros::Time::now();
    robot_marker.ns = "robot_shape";
    robot_marker.id = 0;
    robot_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    robot_marker.action = visualization_msgs::Marker::ADD;
    robot_marker.scale.x = 0.02;
    robot_marker.scale.y = 0.02;
    robot_marker.scale.z = 0.02;
    robot_marker.color.a = 1.0;
    robot_marker.color.r = 0.0;
    robot_marker.color.g = 0.0;
    robot_marker.color.b = 0.0;

    if (env_type_){
        for (const auto &pt : robot_shape_points_){
            geometry_msgs::Point p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            robot_marker.points.push_back(p);
        }
    }
    else {
        for (const auto &pt : robot_shape_points_2d_){
            geometry_msgs::Point p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = odom_base_ptr_->transform.translation.z; // set z to base_link height
            robot_marker.points.push_back(p);
        }
    }
    marker_array.markers.push_back(robot_marker);

    // Add a LINE_STRIP marker to connect the points into a rectangle
    visualization_msgs::Marker rect_marker;
    rect_marker.header.frame_id = robot_marker.header.frame_id;
    rect_marker.header.stamp = robot_marker.header.stamp;
    rect_marker.ns = "robot_shape_rect";
    rect_marker.id = 1;
    rect_marker.type = visualization_msgs::Marker::LINE_LIST;
    rect_marker.action = visualization_msgs::Marker::ADD;
    rect_marker.scale.x = 0.01; // line width
    rect_marker.color.a = 1.0;
    rect_marker.color.r = 0.0;
    rect_marker.color.g = 0.0;
    rect_marker.color.b = 0.0;

    // populate rect_marker points as pairs for LINE_LIST: (a,b) per edge
    size_t n_pts = robot_marker.points.size();
    if (n_pts >= 2) {
        for (size_t i = 0; i < n_pts; ++i) {
            const geometry_msgs::Point &a = robot_marker.points[i];
            const geometry_msgs::Point &b = robot_marker.points[(i + 1) % n_pts];
            rect_marker.points.push_back(a);
            rect_marker.points.push_back(b);
        }
    }

    // only publish rectangle if there are points
    if (!rect_marker.points.empty()) {
        marker_array.markers.push_back(rect_marker);
    }

    // put center point for visualization
    geometry_msgs::Point center_pt;
    center_pt.x = 0.0;
    center_pt.y = 0.0;
    center_pt.z = env_type_ ? 0.0 : odom_base_ptr_->transform.translation.z; // set z to base_link height
    robot_marker.points.push_back(center_pt);
    marker_array.markers.push_back(robot_marker);

    robot_points_pub_.publish(marker_array);
}

void PlannerManager::publishRobotSphere() {
    if (env_type_){
        vec_E<Ellipsoid3D> robot_ellipsoid_vec;
        robot_ellipsoid_vec.push_back(robot_ellipsoid_);
        decomp_ros_msgs::EllipsoidArray ellipsoid_msg = DecompROS::ellipsoid_array_to_ros(robot_ellipsoid_vec);
        ellipsoid_msg.header.stamp = ros::Time::now();
        ellipsoid_msg.header.frame_id = "base_link";
        // publish
        robot_sphere_pub_.publish(ellipsoid_msg);
    }
    else {
        vec_E<Ellipsoid2D> robot_ellipsoid_vec_2d;
        robot_ellipsoid_vec_2d.push_back(robot_ellipsoid_2d_);
        decomp_ros_msgs::EllipsoidArray ellipsoid_msg_2d = DecompROS::ellipsoid_array_to_ros(robot_ellipsoid_vec_2d);
        ellipsoid_msg_2d.header.stamp = ros::Time::now();
        ellipsoid_msg_2d.header.frame_id = "base_link";
        // publish
        robot_sphere_pub_.publish(ellipsoid_msg_2d);
    }
}

void PlannerManager::publishTestCube(){
    // if(current_node_ == nullptr) return;
    // if(current_node_->children.empty()) return;
    if (!free_regions_graph_ptr_) return;
    auto* current_node = free_regions_graph_ptr_->getNode(current_node_id_);
    if (current_node == nullptr) return;
    if (current_node->edge_ids_.empty()) return;

    visualization_msgs::MarkerArray cube_array;
    for(size_t idx = 0; idx < current_node->edge_ids_.size(); ++idx){
        // GraphNode* child_node = current_node_->children[idx];
        EdgeId eid = current_node->edge_ids_[idx];
        auto* e = free_regions_graph_ptr_->getEdge(eid);
        if (!e) continue;

        visualization_msgs::Marker cube_marker;
        cube_marker.header.frame_id = "odom";
        cube_marker.header.stamp = ros::Time::now();
        cube_marker.ns = "child_cube";
        cube_marker.id = static_cast<int>(idx);
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;

        cube_marker.pose.position.x = e->replan_pos_[0];
        cube_marker.pose.position.y = e->replan_pos_[1];
        cube_marker.pose.position.z = env_type_ ? e->replan_pos_[2] : 0.0;
        if (env_type_){
            cube_marker.scale.x = robot_shape_points_[0][0] - robot_shape_points_[2][0];
            cube_marker.scale.y = robot_shape_points_[0][1] - robot_shape_points_[1][1];
            cube_marker.scale.z = robot_shape_points_[0][2] - robot_shape_points_[4][2];
            cube_marker.scale.z = 0.2;
            cube_marker.color.a = 0.5;
            cube_marker.color.r = 1.0;
            cube_marker.color.g = 0.0;
            cube_marker.color.b = 0.0;
            Eigen::Matrix3d R = e->R_;
            Eigen::Quaterniond q(R);
            cube_marker.pose.orientation.x = q.x();
            cube_marker.pose.orientation.y = q.y();
            cube_marker.pose.orientation.z = q.z();
            cube_marker.pose.orientation.w = q.w();
        }
        else{
            cube_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
            cube_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
            cube_marker.scale.z = 0.2;
            cube_marker.color.a = 0.5;
            cube_marker.color.r = 1.0;
            cube_marker.color.g = 0.0;
            cube_marker.color.b = 0.0;
            // from 2D rotation matrix to quaternion (robot only has yaw)
            Eigen::Matrix2d R = e->R_2d_;
            // embed into a 3x3 rotation matrix (rotation about Z)
            Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
            R3.block<2,2>(0,0) = R;
            Eigen::Quaterniond q(R3);
            cube_marker.pose.orientation.x = q.x();
            cube_marker.pose.orientation.y = q.y();
            cube_marker.pose.orientation.z = q.z();
            cube_marker.pose.orientation.w = q.w();
        }

        cube_array.markers.push_back(cube_marker);
    }
    auto single_cube = cube_array;
    single_cube.markers.resize(1); 
    test_cube_pub_.publish(single_cube);
    // test_cube_pub_.publish(cube_array);
}

void PlannerManager::publishTrajectoryForVisualization(BezierSE2 &traj, double worst_violation_time){
    // get 10 points along the traj, and pub robot shape at those points
    visualization_msgs::MarkerArray traj_marker_array;
    int num_points = 9;
    for (int i = 0; i <= num_points; ++i){
        double t = double(i) / double(num_points);
        Eigen::Vector2d pos = traj.pos(t);
        // create a marker for the robot at this position
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "traj_robot";
        robot_marker.id = i;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
        robot_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
        robot_marker.scale.z = 0.2;
        robot_marker.color.a = 0.5;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 1.0;
        robot_marker.pose.position.x = pos[0];
        robot_marker.pose.position.y = pos[1];
        robot_marker.pose.position.z = 0;
        // from 2D rotation matrix to quaternion (robot only has yaw)
        Eigen::Matrix2d R_mat = traj.R(t);
        // embed into a 3x3 rotation matrix (rotation about Z)
        Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
        R3.block<2,2>(0,0) = R_mat;
        Eigen::Quaterniond q(R3);
        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(robot_marker);
    }
    // if worst_violation_time is provided, pub additional marker to indicate it
    if (worst_violation_time >= 0.0){
        visualization_msgs::Marker violation_marker;
        violation_marker.header.frame_id = "odom";
        violation_marker.header.stamp = ros::Time::now();
        violation_marker.ns = "traj_robot";
        violation_marker.id = 0;
        violation_marker.type = visualization_msgs::Marker::CUBE;
        violation_marker.action = visualization_msgs::Marker::ADD;
        violation_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
        violation_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
        violation_marker.scale.z = 0.2;
        violation_marker.color.a = 0.5;
        violation_marker.color.r = 1.0;
        violation_marker.color.g = 0.0;
        violation_marker.color.b = 0.0;
        Eigen::Vector2d pos = traj.pos(worst_violation_time);
        violation_marker.pose.position.x = pos[0];
        violation_marker.pose.position.y = pos[1];
        violation_marker.pose.position.z = 0;
        // from 2D rotation matrix to quaternion (robot only has yaw)
        Eigen::Matrix2d R_mat = traj.R(worst_violation_time);
        // embed into a 3x3 rotation matrix (rotation about Z)
        Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
        R3.block<2,2>(0,0) = R_mat;
        Eigen::Quaterniond q(R3);
        violation_marker.pose.orientation.x = q.x();
        violation_marker.pose.orientation.y = q.y();
        violation_marker.pose.orientation.z = q.z();
        violation_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(violation_marker);
    }
    traj_vis_pub_.publish(traj_marker_array);
}

void PlannerManager::publishTrajectoryAfterOptimization(BezierSE2 &traj){
    // get 10 points along the traj, and pub robot shape at those points
    visualization_msgs::MarkerArray traj_marker_array;
    int num_points = 9;
    for (int i = 0; i <= num_points; ++i){
        double t = double(i) / double(num_points);
        Eigen::Vector2d pos = traj.pos(t);
        // create a marker for the robot at this position
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "traj_robot";
        robot_marker.id = i;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
        robot_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
        robot_marker.scale.z = 0.2;
        robot_marker.color.a = 1.0;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 0.0;
        robot_marker.pose.position.x = pos[0];
        robot_marker.pose.position.y = pos[1];
        robot_marker.pose.position.z = 0;
        // from 2D rotation matrix to quaternion (robot only has yaw)
        Eigen::Matrix2d R_mat = traj.R(t);
        // embed into a 3x3 rotation matrix (rotation about Z)
        Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
        R3.block<2,2>(0,0) = R_mat;
        Eigen::Quaterniond q(R3);
        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(robot_marker);
    }
    traj_after_opt_pub_.publish(traj_marker_array);
}

void PlannerManager::publishTrajectoryAfterOptimization(BezierSE3 &traj){
    // get 10 points along the traj, and pub robot shape at those points
    visualization_msgs::MarkerArray traj_marker_array;
    int num_points = 9;
    for (int i = 0; i <= num_points; ++i){
        double t = double(i) / double(num_points);
        Eigen::Vector3d pos = traj.pos(t);
        // create a marker for the robot at this position
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "traj_robot";
        robot_marker.id = i;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.scale.x = robot_shape_points_[0][0] - robot_shape_points_[2][0];
        robot_marker.scale.y = robot_shape_points_[0][1] - robot_shape_points_[1][1];
        robot_marker.scale.z = robot_shape_points_[0][2] - robot_shape_points_[4][2];
        robot_marker.color.a = 1.0;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 0.0;
        robot_marker.pose.position.x = pos[0];
        robot_marker.pose.position.y = pos[1];
        robot_marker.pose.position.z = pos[2];
        Eigen::Matrix3d R = traj.R(t);
        Eigen::Quaterniond q(R);
        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(robot_marker);
    }
    traj_after_opt_pub_.publish(traj_marker_array);
}

void PlannerManager::publishTrajectoryForVisualization(BezierSE3 &traj, double worst_violation_time){
    // get 10 points along the traj, and pub robot shape at those points
    visualization_msgs::MarkerArray traj_marker_array;
    int num_points = 9;
    for (int i = 0; i <= num_points; ++i){
        double t = double(i) / double(num_points);
        Eigen::Vector3d pos = traj.pos(t);
        // create a marker for the robot at this position
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "traj_robot";
        robot_marker.id = i;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.scale.x = robot_shape_points_[0][0] - robot_shape_points_[2][0];
        robot_marker.scale.y = robot_shape_points_[0][1] - robot_shape_points_[1][1];
        robot_marker.scale.z = robot_shape_points_[0][2] - robot_shape_points_[4][2];
        robot_marker.color.a = 0.5;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 1.0;
        robot_marker.pose.position.x = pos[0];
        robot_marker.pose.position.y = pos[1];
        robot_marker.pose.position.z = pos[2];
        Eigen::Matrix3d R = traj.R(t);
        Eigen::Quaterniond q(R);
        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(robot_marker);
    }
    // if worst_violation_time is provided, pub additional marker to indicate it
    if (worst_violation_time >= 0.0){
        visualization_msgs::Marker violation_marker;
        violation_marker.header.frame_id = "odom";
        violation_marker.header.stamp = ros::Time::now();
        violation_marker.ns = "traj_robot";
        violation_marker.id = 0;
        violation_marker.type = visualization_msgs::Marker::CUBE;
        violation_marker.action = visualization_msgs::Marker::ADD;
        violation_marker.scale.x = robot_shape_points_[0][0] - robot_shape_points_[2][0];
        violation_marker.scale.y = robot_shape_points_[0][1] - robot_shape_points_[1][1];
        violation_marker.scale.z = robot_shape_points_[0][2] - robot_shape_points_[4][2];
        violation_marker.color.a = 0.5;
        violation_marker.color.r = 1.0;
        violation_marker.color.g = 0.0;
        violation_marker.color.b = 0.0;
        Eigen::Vector3d pos = traj.pos(worst_violation_time);
        violation_marker.pose.position.x = pos[0];
        violation_marker.pose.position.y = pos[1];
        violation_marker.pose.position.z = pos[2];
        // from 2D rotation matrix to quaternion (robot only has yaw)
        Eigen::Matrix3d R = traj.R(worst_violation_time);
        Eigen::Quaterniond q(R);
        violation_marker.pose.orientation.x = q.x();
        violation_marker.pose.orientation.y = q.y();
        violation_marker.pose.orientation.z = q.z();
        violation_marker.pose.orientation.w = q.w();
        traj_marker_array.markers.push_back(violation_marker);
    }
    traj_vis_pub_.publish(traj_marker_array);
}

void PlannerManager::publishTrajectoryForVisualizationIter(BezierSE2 &traj, double worst_violation_time, int iter_id)
{
    if (iter_id < 0) return;
    if (iter_id >= (int)traj_iter_pubs_.size()) return;

    visualization_msgs::MarkerArray traj_marker_array;

    const int num_points = 9;
    for (int i = 0; i <= num_points; ++i){
        double t = double(i) / double(num_points);
        Eigen::Vector2d pos = traj.pos(t);

        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "odom";
        robot_marker.header.stamp = ros::Time(0);
        robot_marker.ns = "traj_robot"; // 每个 topic 独立，不怕冲突
        robot_marker.id = i;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;

        robot_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
        robot_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
        robot_marker.scale.z = 0.2;

        robot_marker.color.a = 0.5;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 1.0;

        robot_marker.pose.position.x = pos[0];
        robot_marker.pose.position.y = pos[1];
        robot_marker.pose.position.z = 0;

        Eigen::Matrix2d R_mat = traj.R(t);
        Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
        R3.block<2,2>(0,0) = R_mat;
        Eigen::Quaterniond q(R3);

        robot_marker.pose.orientation.x = q.x();
        robot_marker.pose.orientation.y = q.y();
        robot_marker.pose.orientation.z = q.z();
        robot_marker.pose.orientation.w = q.w();

        traj_marker_array.markers.push_back(robot_marker);
    }

    // worst violation marker
    if (worst_violation_time >= 0.0){
        visualization_msgs::Marker violation_marker;
        violation_marker.header.frame_id = "odom";
        violation_marker.header.stamp = ros::Time(0);
        violation_marker.ns = "traj_robot";
        violation_marker.id = 50;
        violation_marker.type = visualization_msgs::Marker::CUBE;
        violation_marker.action = visualization_msgs::Marker::ADD;

        violation_marker.scale.x = robot_shape_points_2d_[0][0] - robot_shape_points_2d_[2][0];
        violation_marker.scale.y = robot_shape_points_2d_[0][1] - robot_shape_points_2d_[1][1];
        violation_marker.scale.z = 0.5;

        violation_marker.color.a = 0.6;
        violation_marker.color.r = 1.0;
        violation_marker.color.g = 0.0;
        violation_marker.color.b = 0.0;

        Eigen::Vector2d pos = traj.pos(worst_violation_time);
        violation_marker.pose.position.x = pos[0];
        violation_marker.pose.position.y = pos[1];
        violation_marker.pose.position.z = 0;

        Eigen::Matrix2d R_mat = traj.R(worst_violation_time);
        Eigen::Matrix3d R3 = Eigen::Matrix3d::Identity();
        R3.block<2,2>(0,0) = R_mat;
        Eigen::Quaterniond q(R3);

        violation_marker.pose.orientation.x = q.x();
        violation_marker.pose.orientation.y = q.y();
        violation_marker.pose.orientation.z = q.z();
        violation_marker.pose.orientation.w = q.w();

        traj_marker_array.markers.push_back(violation_marker);
    }

    traj_iter_pubs_[iter_id].publish(traj_marker_array);
}

void PlannerManager::publishCurrentDirection(Eigen::Vector3d &start_pos, Eigen::Vector3d &gap_direction) {
    
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker dir_marker;
    dir_marker.header.frame_id = "odom";
    dir_marker.header.stamp = ros::Time::now();
    dir_marker.ns = "current_direction";
    dir_marker.id = 0;
    dir_marker.type = visualization_msgs::Marker::LINE_LIST;
    dir_marker.action = visualization_msgs::Marker::ADD;
    dir_marker.scale.x = 0.02f;        // line width
    dir_marker.color.r = 1.0f;         // magenta
    dir_marker.color.g = 0.0f;
    dir_marker.color.b = 1.0f;
    dir_marker.color.a = 1.0f;

    geometry_msgs::Point start_pt;
    start_pt.x = start_pos[0];
    start_pt.y = start_pos[1];
    start_pt.z = env_type_ ? start_pos[2] : 0.0;

    geometry_msgs::Point end_pt;
    end_pt.x = start_pos[0] + gap_direction[0];
    end_pt.y = start_pos[1] + gap_direction[1];
    end_pt.z = env_type_ ? start_pos[2] + gap_direction[2] : 0.0;

    dir_marker.points.push_back(start_pt);
    dir_marker.points.push_back(end_pt);

    marker_array.markers.push_back(dir_marker);
    current_direction_pub_.publish(marker_array);
}

void PlannerManager::publishSelectedEdgePolyhedron()
{
    if (!free_regions_graph_ptr_) return;
    if (current_edge_id_ < 0) return;

    std::lock_guard<std::mutex> lk(graph_mutex_);

    auto* e = free_regions_graph_ptr_->getEdge(current_edge_id_);
    if (!e) return;

    decomp_ros_msgs::PolyhedronArray poly_msg;
    poly_msg.header.stamp = ros::Time::now();
    poly_msg.header.frame_id = "odom";

    if (env_type_) {
        vec_E<Polyhedron3D> polys;
        polys.push_back(e->corridor_3d_);
        poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    } else {
        vec_E<Polyhedron2D> polys;
        polys.push_back(e->corridor_2d_);
        poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    }

    poly_msg.header.stamp = ros::Time::now();
    poly_msg.header.frame_id = "odom";
    selected_edge_poly_pub_.publish(poly_msg);
}

void PlannerManager::debugTimerCallback(const ros::TimerEvent &event) {
    publishRobotPoints();
    publishRobotSphere();
    // publishTestCube();
    publishCurrentDirection(current_pos, current_direction_for_visualization_);
}

PlannerManager::~PlannerManager()
{
    shutting_down_ = true;

    {
        std::lock_guard<std::mutex> lk(bg_job_mutex_);
        pending_expand_jobs_.clear();
    }

    if (background_expand_thread_.joinable()) {
        background_expand_thread_.join();
    }

    ROS_INFO("[PlannerManager] Destructor: background expansion thread joined.");
}