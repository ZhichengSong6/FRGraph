#include "planner_manager.h"

void PlannerManager::initPlannerModule(ros::NodeHandle &nh) {
    node_ = nh;
    gap_candidates_open_.clear();
    gap_candidates_limited_.clear();
    gap_candidates_free_.clear();

    if(env_type_){
        ROS_INFO("3D environment, using velodyne pointcloud");
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &PlannerManager::velodyneCallback, this);
    }
    else {
        ROS_INFO("2D environment, using 2D laser scan");
        scan2d_sub_ = node_.subscribe("/scan", 1, &PlannerManager::scan2dCallback, this);
    }

    candidate_gaps_sub_ = node_.subscribe("/gap_candidates", 1, &PlannerManager::candidateGapsCallback, this);

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
    if (!getTrajectoryTemp(start_pos, current_node_)) {
        ROS_WARN("[PlannerManager] Failed to generate temporary trajectory.");
        return;
    }
    // print goal pos and the selected replan pos
    ROS_INFO("[PlannerManager] Goal position: (%.2f, %.2f, %.2f)", goal_pos[0], goal_pos[1], goal_pos[2]);
    ROS_INFO("[PlannerManager] Selected replan position: (%.2f, %.2f, %.2f)", current_node_->replan_pos_[0], current_node_->replan_pos_[1], current_node_->replan_pos_[2]);
}