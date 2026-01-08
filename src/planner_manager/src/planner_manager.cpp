#include "planner_manager.h"

void PlannerManager::initPlannerModule(ros::NodeHandle &nh) {
    node_ = nh;
    gap_candidates_open_.clear();
    gap_candidates_limited_.clear();
    gap_candidates_free_.clear();

    if(env_type_){
        ROS_INFO("3D environment, using velodyne pointcloud");
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &PlannerManager::velodyneCallback, this);
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
        std::vector<double> robot_shape_pts_2d;
        node_.param<std::vector<double>>("robot_shape/points_2d", robot_shape_pts_2d, {0.1, 0.1,
                                                                                         0.1, -0.1,
                                                                                        -0.1, -0.1,
                                                                                        -0.1, 0.1});
        // convert to vec_Eigen format
        for (size_t i = 0; i < robot_shape_pts_2d.size(); i += 2) {
            robot_shape_points_2d_.push_back(Eigen::Vector2d(robot_shape_pts_2d[i], robot_shape_pts_2d[i+1]));
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
    poly_pub_aniso_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array_aniso", 1, true);
    robot_points_pub_ = node_.advertise<visualization_msgs::MarkerArray>("planner_manager_robot_points", 1, true);
    robot_sphere_pub_ = node_.advertise<decomp_ros_msgs::EllipsoidArray>("planner_manager_sphere", 1, true);

    poly_pub_aniso_full_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_aniso_full_array", 1, true);
    poly_frtree_pub_ = node_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_frtree_array", 1, true);

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

    free_regions_graph_ptr_.reset(new FreeRegionsGraph());

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

void PlannerManager::decomposeAlongGapDirections(Eigen::Vector3d &start_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates) {
    /* TEST */
    polys_aniso_2d_.clear();
    polys_2d_.clear();
    polys_aniso_3d_.clear();
    polys_3d_.clear();

    polys_aniso_full_2d_.clear();
    if (env_type_){
        const Vec3f p1(start_pos[0], start_pos[1], start_pos[2]);
        const Vec3f p2(all_candidates[0].dir_odom_frame[0],
                   all_candidates[0].dir_odom_frame[1],
                   all_candidates[0].dir_odom_frame[2]);
        LineSegment3D line_segment(p1, p2);
        line_segment.set_obs(pointcloud_cropped_odom_frame_);
        line_segment.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f)); // set local bbox for decomposition
        line_segment.dilate(-0.1f);

        LineSegment3D line_segment_aniso(p1, p2);
        line_segment_aniso.set_obs(pointcloud_cropped_odom_frame_);
        line_segment_aniso.set_local_bbox_aniso(Vec3f(0.0f, 0.5f, 0.5f),
                                                Vec3f(0.3f, 0.5f, 0.5f)); // set local bbox for decomposition

        // get robot shape in odom frame
        Eigen::Vector3d base_pos_odom(0.0, 0.0, 0.0);
        Eigen::Quaterniond base_rot(1.0, 0.0, 0.0, 0.0);
        if (odom_base_ptr_) {
            base_pos_odom[0] = odom_base_ptr_->transform.translation.x;
            base_pos_odom[1] = odom_base_ptr_->transform.translation.y;
            base_pos_odom[2] = odom_base_ptr_->transform.translation.z;
            base_rot = Eigen::Quaterniond(
                odom_base_ptr_->transform.rotation.w,
                odom_base_ptr_->transform.rotation.x,
                odom_base_ptr_->transform.rotation.y,
                odom_base_ptr_->transform.rotation.z);
        }
        const Eigen::Matrix3d base_rot_mat = base_rot.normalized().toRotationMatrix();
        std::vector<Vec3f> robot_shape_points_odom;
        for (const auto &pt : robot_shape_points_) {
            const Eigen::Vector3d local_pt(pt[0], pt[1], pt[2]);
            const Eigen::Vector3d rotated_pt = base_rot_mat * local_pt + base_pos_odom;
            robot_shape_points_odom.emplace_back(
                static_cast<float>(rotated_pt[0]),
                static_cast<float>(rotated_pt[1]),
                static_cast<float>(rotated_pt[2]));
        }
        line_segment_aniso.set_robot_shape_pts(robot_shape_points_odom);

        const Eigen::Vector3d center_base = robot_ellipsoid_.d().cast<double>();
        const Eigen::Vector4d center_base_homo(center_base[0], center_base[1], center_base[2], 1.0);
        const Eigen::Matrix4d Tmat = tf2::transformToEigen(*odom_base_ptr_).matrix();
        const Eigen::Vector4d center_odom_homo = Tmat * center_base_homo;
        const Eigen::Vector3d center_odom = center_odom_homo.head<3>();

        const double radius = robot_ellipsoid_.C()(0,0);
        line_segment_aniso.dilate_aniso(Vec3f(center_odom[0], center_odom[1], center_odom[2]), radius);

        polys_aniso_3d_.push_back(line_segment_aniso.get_polyhedron());
        polys_3d_.push_back(line_segment.get_polyhedron());
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_3d_);
        decomp_ros_msgs::PolyhedronArray poly_msg_aniso = DecompROS::polyhedron_array_to_ros(polys_aniso_3d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_pub_.publish(poly_msg);
        poly_msg_aniso.header.stamp = ros::Time::now();
        poly_msg_aniso.header.frame_id = "odom";
        poly_pub_aniso_.publish(poly_msg_aniso);
    }
    else{
        // 2D case
        const Vec2f p1(start_pos[0], start_pos[1]);
        const Vec2f p2(all_candidates[0].dir_odom_frame[0],
                     all_candidates[0].dir_odom_frame[1]);
        LineSegment2D line_segment(p1, p2);
        line_segment.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment.set_local_bbox(Vec2f(0.5f, 0.5f)); // set local bbox for decomposition
    // auto t0 = std::chrono::high_resolution_clock::now();
        line_segment.dilate(0.1f);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
    // ROS_INFO("[PlannerManager] dilate elapsed: %.3f ms", ms);

        LineSegment2D line_segment_aniso(p1, p2);
        line_segment_aniso.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment_aniso.set_local_bbox_aniso(Vec2f(0.0f, 0.5f),
                                               Vec2f(0.3f, 0.5f)); // set local bbox for decomposition

        // get robot shape in odom frame
        Eigen::Vector2d base_pos_odom(0.0, 0.0);
        Eigen::Quaterniond base_rot(1.0, 0.0, 0.0, 0.0);
        if (odom_base_ptr_) {
            base_pos_odom[0] = odom_base_ptr_->transform.translation.x;
            base_pos_odom[1] = odom_base_ptr_->transform.translation.y;
            base_rot = Eigen::Quaterniond(
                odom_base_ptr_->transform.rotation.w,
                odom_base_ptr_->transform.rotation.x,
                odom_base_ptr_->transform.rotation.y,
                odom_base_ptr_->transform.rotation.z);
        }
        const Eigen::Matrix3d base_rot_mat = base_rot.normalized().toRotationMatrix();
        const Eigen::Matrix2d base_rot_2d = base_rot_mat.block<2, 2>(0, 0);
        std::vector<Vec2f> robot_shape_points_odom;
        robot_shape_points_odom.reserve(robot_shape_points_2d_.size());
        for (const auto &pt : robot_shape_points_2d_) {
            const Eigen::Vector2d local_pt(pt[0], pt[1]);
            const Eigen::Vector2d rotated_pt = base_rot_2d * local_pt + base_pos_odom;
            robot_shape_points_odom.emplace_back(
                static_cast<float>(rotated_pt[0]),
                static_cast<float>(rotated_pt[1]));
        }
        line_segment_aniso.set_robot_shape_pts(robot_shape_points_odom);
        const Eigen::Vector2d center_base = robot_ellipsoid_2d_.d().cast<double>();
        const Eigen::Vector3d center_base_homo(center_base[0], center_base[1], 0.0);
        const Eigen::Affine3d T_odom_base = tf2::transformToEigen(*odom_base_ptr_);
        const Eigen::Vector3d center_odom = T_odom_base * center_base_homo;

        const double radius = robot_ellipsoid_2d_.C()(0, 0);
    // auto t2 = std::chrono::high_resolution_clock::now();
    line_segment_aniso.dilate_aniso(Vec2f(center_odom[0], center_odom[1]), static_cast<float>(radius));
    // auto t3 = std::chrono::high_resolution_clock::now();
    // double ms_1 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t3 - t2).count();
    // ROS_INFO("[PlannerManager] dilate_aniso elapsed: %.3f ms", ms_1);

        LineSegment2D line_segment_aniso_full(p1, p2);
        line_segment_aniso_full.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment_aniso_full.set_local_bbox_aniso(Vec2f(0.0f, 0.3f),
                                                    Vec2f(0.3f, 0.3f)); // set local bbox for decomposition
        line_segment_aniso_full.set_robot_shape_pts(robot_shape_points_odom);
        line_segment_aniso_full.dilate_aniso_full(Vec2f(center_odom[0], center_odom[1]), static_cast<float>(radius));

        polys_aniso_full_2d_.push_back(line_segment_aniso_full.get_polyhedron());
        polys_aniso_2d_.push_back(line_segment_aniso.get_polyhedron());
        polys_2d_.push_back(line_segment.get_polyhedron());
        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_2d_);
        decomp_ros_msgs::PolyhedronArray poly_msg_aniso = DecompROS::polyhedron_array_to_ros(polys_aniso_2d_);
        decomp_ros_msgs::PolyhedronArray poly_msg_aniso_full = DecompROS::polyhedron_array_to_ros(polys_aniso_full_2d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_pub_.publish(poly_msg);
        poly_msg_aniso.header.stamp = ros::Time::now();
        poly_msg_aniso.header.frame_id = "odom";
        poly_pub_aniso_.publish(poly_msg_aniso);
        poly_msg_aniso_full.header.stamp = ros::Time::now();
        poly_msg_aniso_full.header.frame_id = "odom";
        poly_pub_aniso_full_.publish(poly_msg_aniso_full);
    }
}

void PlannerManager::decomposeAlongGapDirections_FRTree(Eigen::Vector3d &start_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates) {
    polys_FRTree_2d_.clear();
    polys_FRTree_3d_.clear();
    if (!env_type_){
        Eigen::Vector2d dir = all_candidates[0].dir_odom_frame.head<2>() - start_pos.head<2>();
        double dist = dir.norm();
        dir.normalize();
        Eigen::Vector2d pos1, pos2, pos3, pos4, pos5, pos6;
        pos1 = start_pos.head<2>() + dir * (dist * 0.05);
        pos2 = start_pos.head<2>() + dir * (dist * 0.45);

        pos3 = start_pos.head<2>() + dir * (dist * 0.4);
        pos4 = start_pos.head<2>() + dir * (dist * 0.7);

        pos5 = start_pos.head<2>() + dir * (dist * 0.65);
        pos6 = start_pos.head<2>() + dir * (dist * 0.9);

        const Vec2f p1(pos1[0], pos1[1]);
        const Vec2f p2(pos2[0], pos2[1]);
        const Vec2f p3(pos3[0], pos3[1]);
        const Vec2f p4(pos4[0], pos4[1]);
        const Vec2f p5(pos5[0], pos5[1]);
        const Vec2f p6(pos6[0], pos6[1]);

        LineSegment2D line_segment1(p1, p2);
        line_segment1.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment1.set_local_bbox(Vec2f(0.5f, 0.5f)); // set local bbox for decomposition
        line_segment1.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment1.get_polyhedron());
        LineSegment2D line_segment2(p3, p4);
        line_segment2.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment2.set_local_bbox(Vec2f(0.5f, 0.5f)); // set local bbox for decomposition
        line_segment2.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment2.get_polyhedron());
        LineSegment2D line_segment3(p5, p6);
        line_segment3.set_obs(pointcloud_cropped_odom_frame_2d_);
        line_segment3.set_local_bbox(Vec2f(0.5f, 0.5f)); // set local bbox for decomposition
        line_segment3.dilate(0.1f);
        polys_FRTree_2d_.push_back(line_segment3.get_polyhedron()); 

        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_FRTree_2d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_frtree_pub_.publish(poly_msg);
    }
    else{
        // 3D
        Eigen::Vector3d dir = all_candidates[0].dir_odom_frame - start_pos;
        double dist = dir.norm();
        dir.normalize();
        Eigen::Vector3d pos1, pos2, pos3, pos4, pos5, pos6;
        pos1 = start_pos + dir * (dist * 0.05);
        pos2 = start_pos + dir * (dist * 0.45);

        pos3 = start_pos + dir * (dist * 0.4);
        pos4 = start_pos + dir * (dist * 0.7);

        pos5 = start_pos + dir * (dist * 0.65);
        pos6 = start_pos + dir * (dist * 0.9);

        const Vec3f p1(pos1[0], pos1[1], pos1[2]);
        const Vec3f p2(pos2[0], pos2[1], pos2[2]);
        const Vec3f p3(pos3[0], pos3[1], pos3[2]);
        const Vec3f p4(pos4[0], pos4[1], pos4[2]);
        const Vec3f p5(pos5[0], pos5[1], pos5[2]);
        const Vec3f p6(pos6[0], pos6[1], pos6[2]);

        LineSegment3D line_segment1(p1, p2);
        line_segment1.set_obs(pointcloud_cropped_odom_frame_);
        line_segment1.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f)); // set local bbox for decomposition
        line_segment1.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment1.get_polyhedron());
        LineSegment3D line_segment2(p3, p4);
        line_segment2.set_obs(pointcloud_cropped_odom_frame_);
        line_segment2.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f)); // set local bbox for decomposition
        line_segment2.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment2.get_polyhedron());
        LineSegment3D line_segment3(p5, p6);
        line_segment3.set_obs(pointcloud_cropped_odom_frame_);
        line_segment3.set_local_bbox(Vec3f(0.5f, 0.5f, 0.5f)); // set local bbox for decomposition
        line_segment3.dilate(0.1f);
        polys_FRTree_3d_.push_back(line_segment3.get_polyhedron());

        decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys_FRTree_3d_);
        poly_msg.header.stamp = ros::Time::now();
        poly_msg.header.frame_id = "odom";
        poly_frtree_pub_.publish(poly_msg);
    }
}

void PlannerManager::sortAllCandidatesGap(Eigen:: Vector3d &goal_pos, 
                                          std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates) {
    // first, retain all the candidates gaps
    all_candidates.clear();
    all_candidates.reserve(gap_candidates_open_.size() +
                           gap_candidates_limited_.size() +
                           gap_candidates_free_.size());
    all_candidates.insert(all_candidates.end(), gap_candidates_open_.begin(),    gap_candidates_open_.end());
    all_candidates.insert(all_candidates.end(), gap_candidates_limited_.begin(), gap_candidates_limited_.end());
    all_candidates.insert(all_candidates.end(), gap_candidates_free_.begin(),    gap_candidates_free_.end());

    if (all_candidates.empty()) {
        ROS_WARN("[PlannerManager] No gap candidates available for sorting.");
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

    // compute distance to goal for each gap candidate
    const size_t N = all_candidates.size();
    std::vector<double> distances_to_goal(N, std::numeric_limits<double>::infinity());
    std::vector<size_t> order(N);
    std::iota(order.begin(), order.end(), 0);

    for (size_t i = 0; i < N; ++i) {
        const Eigen::Vector3d dir_odom_frame = to_odom_frame(all_candidates[i].dir_scan_frame);
        all_candidates[i].dir_odom_frame = dir_odom_frame;
        distances_to_goal[i] = (goal_pos - dir_odom_frame).norm();
    }

    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) {
                  return distances_to_goal[a] < distances_to_goal[b];
              });
    // Apply the computed order to all_candidates so the caller receives
    // the list sorted by distance to the goal (ascending).
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> sorted_candidates;
    sorted_candidates.reserve(N);
    std::vector<double> sorted_distances;
    sorted_distances.reserve(N);
    for (size_t idx : order) {
        sorted_candidates.push_back(all_candidates[idx]);
        sorted_distances.push_back(distances_to_goal[idx]);
    }
    all_candidates.swap(sorted_candidates);
    distances_to_goal.swap(sorted_distances);
}

void PlannerManager::reorderCandidatesGapWithGoal(Eigen::Vector3d &goal_pos, std::vector<Gaps, Eigen::aligned_allocator<Gaps>> &all_candidates){
    if (all_candidates.empty()) return;
    gap_extractor_ptr_->checkGoalStatus(goal_pos);
    GoalStatus goal_status = gap_extractor_ptr_->getGoalStatus();
    if (goal_status == GoalStatus::BLOCKED || goal_status == GoalStatus::OUT_OF_VIEW){
        // no need to reorder
        return;
    }

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

bool PlannerManager::getTrajectoryTemp(Eigen::Vector3d &start_pos, GraphNode* current_node) {
    // from current pos to current_node->replan_pos_
    // get 10 points along the straight line
    trajectory_points_temp_.clear();
    const Eigen::Vector3d direction = current_node->replan_pos_ - start_pos;
    const double distance = direction.norm();
    if (distance < 0.1) {
        ROS_WARN("[PlannerManager] Start position is too close to the replan position.");
        return false;
    }
    const Eigen::Vector3d unit_direction = direction / distance;
    const int num_points = 10;
    for (int i = 1; i <= num_points; ++i) {
        Eigen::Vector3d point = start_pos + unit_direction * (distance * i / num_points);
        trajectory_points_temp_.push_back(point);
    }
    return true;
}

void PlannerManager::planTrajectory(Eigen::Vector3d &start_pos, Eigen::Vector3d &goal_pos, GraphNode* current_node) {
    std::vector<Gaps, Eigen::aligned_allocator<Gaps>> all_candidates;
    sortAllCandidatesGap(goal_pos, all_candidates);
    if (all_candidates.empty()) {
        ROS_WARN("[PlannerManager] No gap candidates available for planning.");
        return;
    }

    // test
    decomposeAlongGapDirections(start_pos, all_candidates);
    decomposeAlongGapDirections_FRTree(start_pos, all_candidates);

    reorderCandidatesGapWithGoal(goal_pos, all_candidates);

    current_node->children.clear();
    // push all candidates as children of the current node
    for (const auto &gap : all_candidates) {
        GraphNode* new_node = new GraphNode();
        new_node->parent   = current_node;
        new_node->children.clear();
        new_node->visited  = false;
        new_node->replan_pos_ = gap.dir_odom_frame;
        current_node->children.push_back(new_node);
    }

    for (int i = 0; i < current_node->children.size(); ++i) {
        graph_points_for_visualization_.push_back(current_node->replan_pos_);
        graph_points_for_visualization_.push_back(current_node->children[i]->replan_pos_);
    }

    // choose the best candidate which has not been visited
    for (const auto &child_node : current_node->children) {
        if (!child_node->visited) {
            ROS_INFO("[PlannerManager] New node selected for replanning.");
            child_node->visited = true;
            current_node_ = child_node;
            break;
        }
    }
    // get trajectory
    // if (!getTrajectoryTemp(start_pos, current_node_)) {
    //     ROS_WARN("[PlannerManager] Failed to generate temporary trajectory.");
    //     return;
    // }
    // print goal pos and the selected replan pos
    ROS_INFO("[PlannerManager] Goal position: (%.2f, %.2f, %.2f)", goal_pos[0], goal_pos[1], goal_pos[2]);
    ROS_INFO("[PlannerManager] Selected replan position: (%.2f, %.2f, %.2f)", current_node_->replan_pos_[0], current_node_->replan_pos_[1], current_node_->replan_pos_[2]);
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
    robot_marker.scale.x = 0.05;
    robot_marker.scale.y = 0.05;
    robot_marker.scale.z = 0.05;
    robot_marker.color.a = 1.0;
    robot_marker.color.r = 0.0;
    robot_marker.color.g = 1.0;
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

void PlannerManager::debugTimerCallback(const ros::TimerEvent &event) {
    publishRobotPoints();
    publishRobotSphere();
}