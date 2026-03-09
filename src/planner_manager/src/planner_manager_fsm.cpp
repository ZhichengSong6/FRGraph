#include "planner_manager_fsm.h"

static BezierSE2 reverseBezierSE2(const BezierSE2& traj)
{
    BezierSE2 rev;
    for (int i = 0; i < BezierSE2::K; ++i) {
        rev.P[i] = traj.P[BezierSE2::K - 1 - i];
        rev.Theta[i] = traj.Theta[BezierSE2::K - 1 - i];
    }
    return rev;
}

static BezierSE3 reverseBezierSE3(const BezierSE3& traj)
{
    BezierSE3 rev;
    for (int i = 0; i < BezierSE3::K; ++i) {
        rev.P[i] = traj.P[BezierSE3::K - 1 - i];
        rev.Theta[i] = traj.Theta[BezierSE3::K - 1 - i];
    }
    return rev;
}

void PlannerManagerFSM::init(ros::NodeHandle &nh) {
    node_ = nh;
    current_state_ = INIT;
    have_odom_ = false;
    have_goal_ = false;

    node_.param<int>("env_type", env_type_, 1); // 0 for 2D, 1 for 3D environment
    node_.param<double>("goal_z_pos", goal_z_pos_, 0.5); // default goal z pos for 3D environment
    ROS_INFO("[PlannerManagerFSM] env_type: %d", env_type_);
    std::vector<double> crop_size_vec;
    node_.param<std::vector<double>>("size_of_cropped_pointcloud", crop_size_vec, {3, 3, 2});
    if (crop_size_vec.size() == 3) {
        size_of_cropped_pointcloud_ = Eigen::Vector3d(crop_size_vec[0], crop_size_vec[1], crop_size_vec[2]);
    } else {
        size_of_cropped_pointcloud_ = Eigen::Vector3d(3, 3, 2); // 默认值
        ROS_WARN("size_of_cropped_pointcloud param size error, using default (3,3,2)");
    }

    planner_manager_.reset(new PlannerManager());
    planner_manager_->setEnvType(env_type_);
    planner_manager_->setSizeOfCroppedPointcloud(size_of_cropped_pointcloud_);
    planner_manager_->initPlannerModule(node_);

    /* Callback Function */
    FSM_timer_ = node_.createTimer(ros::Duration(0.2), &PlannerManagerFSM::FSMCallback, this);
    cmd_timer_ = node_.createTimer(ros::Duration(0.2), &PlannerManagerFSM::publishCmdCallback, this);
    replan_check_timer_ = node_.createTimer(ros::Duration(1.0), &PlannerManagerFSM::replanCheckCallback, this);
    visualization_timer_ = node_.createTimer(ros::Duration(1.0), &PlannerManagerFSM::visualizationCallback, this);

    /* ROS subscribers */
    odom_sub_ = node_.subscribe("/odom", 1, &PlannerManagerFSM::odomCallback, this);
    goal_sub_ = node_.subscribe("/goal", 1, &PlannerManagerFSM::goalCallback, this);

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    goal_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/goal_marker", 1);
    global_graph_pub_ = node_.advertise<visualization_msgs::Marker>("/global_graph", 10);
}

void PlannerManagerFSM::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odom_omega_ = Eigen::Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    odom_ori_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    quaternionToRPY(odom_ori_, odom_roll_, odom_pitch_, odom_yaw_);

    have_odom_ = true;
}

void PlannerManagerFSM::goalCallback(const geometry_msgs::PoseStampedPtr &msg) {
    if (!have_odom_){
        return;
    }
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(env_type_ == 1){
        goal_pos_[2] = goal_z_pos_; // set goal z pos for 3D
    }
    else{
        goal_pos_[2] = odom_pos_[2]; // for 2D, keep the same z as current odom
    }
    goal_ori_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    quaternionToRPY(goal_ori_, goal_roll_, goal_pitch_, goal_yaw_);

    stopRobot();
    ROS_INFO("[FSM]: Received new goal");
    changeFSMState(PLAN_TRAJECTORY, "FSM");
    have_goal_ = true;
}

void PlannerManagerFSM::publishCmdCallback(const ros::TimerEvent &e)
{
    if (current_state_ == INIT || current_state_ == WAIT_GOAL) {
        return;
    }

    const EdgeId eid = planner_manager_->current_edge_id_;
    if (eid < 0) {
        ROS_WARN_THROTTLE(2.0, "[FSM]: No current edge to execute!");
        return;
    }

    auto* edge = planner_manager_->free_regions_graph_ptr_->getEdge(eid);
    if (!edge) {
        ROS_WARN_THROTTLE(2.0, "[FSM]: current edge pointer invalid!");
        return;
    }

    if (!edge->has_traj_) {
        ROS_WARN_THROTTLE(2.0, "[FSM]: Edge has no cached trajectory!");
        return;
    }

    // Current robot state in odom
    const Eigen::Vector3d p_w = odom_pos_;
    const Eigen::Matrix3d R_wb = odom_ori_.toRotationMatrix();
    const Eigen::Matrix3d R_bw = R_wb.transpose();

    // Measured velocities in base frame
    const Eigen::Vector3d v_b = odom_vel_;
    const Eigen::Vector3d w_meas_b = odom_omega_;

    geometry_msgs::Twist cmd;

    // gains & limits
    const double vmax_xy = 0.8;
    const double vmax_z  = 0.5;
    const double wmax    = 1.5;
    const double lookahead_s = 0.6;

    const double kp_pos = 1.5;
    const double kd_pos = 0.4;

    const double kp_yaw = 2.0;
    const double kd_yaw = 0.2;

    const double kp_rot = 2.5;
    const double kd_rot = 0.3;

    // ------------------------------------------------------------------
    // Determine whether the current edge trajectory should be followed
    // in forward direction or reverse direction.
    //
    // EXPAND_EDGE: always use forward stored trajectory.
    // PATH_EDGE:
    //   - if current_node_id_ == edge->from_ -> forward
    //   - if current_node_id_ == edge->to_   -> reverse
    // ------------------------------------------------------------------
    bool reverse_traj = false;

    if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::EXPAND_EDGE) {
        reverse_traj = false;
    }
    else if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::PATH_EDGE) {
        const NodeId cur_nid = planner_manager_->current_node_id_;
        if (cur_nid == edge->from_) {
            reverse_traj = false;
        }
        else if (cur_nid == edge->to_) {
            reverse_traj = true;
        }
        else {
            ROS_WARN_THROTTLE(2.0, "[FSM]: PATH_EDGE but current node is not an endpoint of current edge.");
            return;
        }
    }
    else {
        ROS_WARN_THROTTLE(2.0, "[FSM]: current_edge_exec_type_ is NONE while publishing command.");
        return;
    }

    // ============================ SE2 ============================
    if (!env_type_) {
        BezierSE2 traj = reverse_traj ? reverseBezierSE2(edge->traj2_) : edge->traj2_;

        const Eigen::Vector2d p2_w(p_w.x(), p_w.y());

        // 1) project to curve and pick lookahead parameter
        const double t_star = projectToBezierSE2(traj, p2_w, 80);
        const Eigen::Vector2d dp = traj.dpos(t_star);
        const double speed_ref = std::max(0.2, dp.norm());
        double dt = lookahead_s / speed_ref;
        dt = clampd(dt, 0.02, 0.25);
        const double t_ref = clampd(t_star + dt, 0.0, 1.0);

        // 2) reference pose
        const Eigen::Vector2d pref_w = traj.pos(t_ref);
        const double yaw_ref = traj.yaw(t_ref);

        const double yaw = std::atan2(R_wb(1,0), R_wb(0,0));

        // 3) errors in base frame
        const Eigen::Vector2d epos_w = pref_w - p2_w;
        const Eigen::Vector2d epos_b = R_bw.block<2,2>(0,0) * epos_w;

        const double eyaw = wrapToPi(yaw_ref - yaw);

        // 4) PD command
        Eigen::Vector2d vcmd_b = kp_pos * epos_b - kd_pos * v_b.head<2>();
        double wz = kp_yaw * eyaw - kd_yaw * w_meas_b.z();

        // 5) clamp
        vcmd_b.x() = clampd(vcmd_b.x(), -vmax_xy, vmax_xy);
        vcmd_b.y() = clampd(vcmd_b.y(), -vmax_xy, vmax_xy);
        wz = clampd(wz, -wmax, wmax);

        cmd.linear.x = vcmd_b.x();
        cmd.linear.y = vcmd_b.y();
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = wz;

        cmd_vel_pub_.publish(cmd);
        return;
    }

    // ============================ SE3 ============================
    {
        BezierSE3 traj = reverse_traj ? reverseBezierSE3(edge->traj3_) : edge->traj3_;

        // 1) project & lookahead
        const double t_star = projectToBezierSE3(traj, p_w, 80);
        const Eigen::Vector3d dp = traj.dpos(t_star);
        const double speed_ref = std::max(0.2, dp.norm());
        double dt = lookahead_s / speed_ref;
        dt = clampd(dt, 0.02, 0.25);
        const double t_ref = clampd(t_star + dt, 0.0, 1.0);

        // 2) reference pose
        const Eigen::Vector3d pref_w = traj.pos(t_ref);
        const Eigen::Matrix3d Rref_wb = traj.R(t_ref);

        // 3) errors in base frame
        const Eigen::Vector3d epos_w = pref_w - p_w;
        const Eigen::Vector3d epos_b = R_bw * epos_w;

        const Eigen::Matrix3d R_err = R_bw * Rref_wb;
        const Eigen::Vector3d e_rot_b = so3Log(R_err);

        // 4) PD commands
        Eigen::Vector3d vcmd_b = kp_pos * epos_b - kd_pos * v_b;
        Eigen::Vector3d wcmd_b = kp_rot * e_rot_b - kd_rot * w_meas_b;

        // 5) clamp
        vcmd_b.x() = clampd(vcmd_b.x(), -vmax_xy, vmax_xy);
        vcmd_b.y() = clampd(vcmd_b.y(), -vmax_xy, vmax_xy);
        vcmd_b.z() = clampd(vcmd_b.z(), -vmax_z,  vmax_z);

        wcmd_b.x() = clampd(wcmd_b.x(), -wmax, wmax);
        wcmd_b.y() = clampd(wcmd_b.y(), -wmax, wmax);
        wcmd_b.z() = clampd(wcmd_b.z(), -wmax, wmax);

        cmd.linear.x = vcmd_b.x();
        cmd.linear.y = vcmd_b.y();
        cmd.linear.z = vcmd_b.z();
        cmd.angular.x = wcmd_b.x();
        cmd.angular.y = wcmd_b.y();
        cmd.angular.z = wcmd_b.z();

        cmd_vel_pub_.publish(cmd);
        return;
    }
}

void PlannerManagerFSM::replanCheckCallback(const ros::TimerEvent &e) {
    // a callback to check if need to replan
    // A: achieved subgoal
    // B: no more gap above (TBD)
    if (current_state_ != EXEC_TRAJECTORY){
        return;
    }

    EdgeId eid = planner_manager_->current_edge_id_;
    if (eid < 0) {
        ROS_WARN_THROTTLE(2.0, "[FSM]: current_edge_id_ invalid.");
        return;
    }
    auto* edge = planner_manager_->free_regions_graph_ptr_->getEdge(eid);

    if (!edge) {
        ROS_WARN("Edge pointer invalid.");
        return;
    }
    // pos error
    double distance_to_subgoal = 0.0;
    // orientation error
    double angle_to_subgoal = 0.0;
    
    if (env_type_){
        distance_to_subgoal = (odom_pos_ - edge->replan_pos_).norm();
        // 3D
        const Eigen::Matrix3d R_cur = odom_ori_.toRotationMatrix();
        const Eigen::Matrix3d R_ref = edge->R_;
        const Eigen::Matrix3d R_err = R_cur.transpose() * R_ref;
        angle_to_subgoal = so3Angle(R_err);
    }
    else{
        distance_to_subgoal = (odom_pos_.head<2>() - edge->replan_pos_.head<2>()).norm();
        Eigen::Matrix2d R_cur;
        R_cur << std::cos(odom_yaw_), -std::sin(odom_yaw_),
                 std::sin(odom_yaw_),  std::cos(odom_yaw_);
        Eigen::Matrix2d R_ref = edge->R_2d_;
        Eigen::Matrix2d R_err = R_cur.transpose() * R_ref;
        angle_to_subgoal = so2Angle(R_err);
    }

    // thresholds
    const double pos_threshold = 0.1; // meters
    double angle_threshold;
    if(env_type_){
        angle_threshold = 15.0 * M_PI / 180.0; // 15 degrees for 3D
    }
    else{
        angle_threshold = 10.0 * M_PI / 180.0; // 10 degrees for 2D
    }
    if (distance_to_subgoal < pos_threshold && angle_to_subgoal < angle_threshold){
        ROS_INFO("[FSM]: Reached subgoal, pos=%.2f m, ang=%.2f deg. Replanning...",
                 distance_to_subgoal, angle_to_subgoal * 180.0 / M_PI);
        // ============================================================
        // Case 1: current edge is a graph path edge
        // ============================================================
        if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::PATH_EDGE) {

            NodeId cur_nid = planner_manager_->current_node_id_;
            NodeId next_nid = planner_manager_->otherEndpoint(eid, cur_nid);
            if (next_nid < 0) {
                ROS_WARN("[FSM]: PATH_EDGE reached, but failed to find the other endpoint.");
                stopRobot();
                return;
            }

            // Move to the already-realized node at the other endpoint
            planner_manager_->current_node_id_ = next_nid;

            // Advance along the precomputed graph path
            planner_manager_->planned_path_index_++;

            // --------------------------------------------------------
            // Still have more path edges to follow:
            // keep EXEC_TRAJECTORY, do not go back to PLAN_TRAJECTORY
            // --------------------------------------------------------
            if (planner_manager_->planned_path_index_ < planner_manager_->planned_path_edges_.size()) {
                planner_manager_->current_edge_id_ =
                    planner_manager_->planned_path_edges_[planner_manager_->planned_path_index_];
                planner_manager_->current_edge_exec_type_ = PlannerManager::EdgeExecType::PATH_EDGE;

                ROS_INFO("[FSM]: Continue along graph path. next path edge = %d",
                        planner_manager_->current_edge_id_);

                // IMPORTANT:
                // stay in EXEC_TRAJECTORY
                // later the controller should directly use the cached bezier of this edge
                return;
            }

            // --------------------------------------------------------
            // Path finished: now we are at the target frontier node.
            // Next step is to execute the chosen expand edge.
            // We go back to PLAN_TRAJECTORY, but this time only to
            // prepare trajectory for the already selected expand edge.
            // --------------------------------------------------------
            planner_manager_->current_edge_id_ = planner_manager_->target_expand_edge_id_;
            planner_manager_->current_edge_exec_type_ = PlannerManager::EdgeExecType::EXPAND_EDGE;

            ROS_INFO("[FSM]: Graph path finished. Switch to expand edge = %d at frontier node %d",
                    planner_manager_->current_edge_id_,
                    planner_manager_->target_frontier_node_id_);

            changeFSMState(PLAN_TRAJECTORY, "replanCheckCallback(PATH_EDGE finished)");
            return;
        }

        // ============================================================
        // Case 2: current edge is an expand edge
        // ============================================================
        if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::EXPAND_EDGE) {

            NodeId nid = -1;

            // Realize a new node at current robot pose
            if (env_type_) {
                Polyhedron3D poly; // Update the poly in expandNode
                nid = planner_manager_->free_regions_graph_ptr_->upsertNode(odom_pos_, poly);
            }
            else {
                Polyhedron2D poly; // Update the poly in expandNode
                nid = planner_manager_->free_regions_graph_ptr_->upsertNode(odom_pos_.head<2>(), poly);
            }

            if (nid < 0) {
                ROS_WARN("[FSM]: Failed to realize a new node after expand edge.");
                stopRobot();
                return;
            }

            // Bind this expand edge to the newly realized node
            edge->to_ = nid;
            planner_manager_->current_node_id_ = nid;

            if (auto* new_node = planner_manager_->free_regions_graph_ptr_->getNode(nid)) {
                new_node->incoming_edge_id_ = eid;
                new_node->edge_ids_.push_back(eid);
            }

            // Reset current execution state
            planner_manager_->current_edge_id_ = -1;
            planner_manager_->current_edge_exec_type_ = PlannerManager::EdgeExecType::NONE;

            // Clear previous frontier/path bookkeeping;
            // a new node means a new planning round.
            planner_manager_->planned_path_edges_.clear();
            planner_manager_->planned_path_index_ = 0;
            planner_manager_->target_frontier_node_id_ = -1;
            planner_manager_->target_expand_edge_id_ = -1;

            changeFSMState(PLAN_TRAJECTORY, "replanCheckCallback(EXPAND_EDGE)");
            return;
        }

        // ============================================================
        // Fallback
        // ============================================================
        ROS_WARN("[FSM]: current_edge_exec_type_ is NONE or unknown at subgoal.");
        planner_manager_->current_edge_id_ = -1;
        planner_manager_->current_edge_exec_type_ = PlannerManager::EdgeExecType::NONE;
        stopRobot();
    }
}

void PlannerManagerFSM::FSMCallback(const ros::TimerEvent &e) {
    /* init phase wating for odom & goal */
    static int FSM_num = 0;
    FSM_num++;
    if (FSM_num == 100){
        if (!have_odom_){
            ROS_INFO("[FSM]: Wait for odom!");
        }
        if (!have_goal_){
            ROS_INFO("[FSM]: Wait for goal!");
        }
        printFSMCurrentState();
        FSM_num = 0;
    }

    switch (current_state_)
    {
        case INIT:
        {
            if (!have_odom_){
                return;
            }
            // initialize the graph with current pos as root node
            if(!graph_inited_){
                Eigen::Vector3d start_pos = odom_pos_;
                planner_manager_->graph_points_for_visualization_.clear();
                planner_manager_->free_regions_graph_ptr_->setRootNode(start_pos);
                planner_manager_->current_node_id_ = planner_manager_->free_regions_graph_ptr_->root_id_;
                planner_manager_->current_edge_id_ = -1;
                planner_manager_->current_edge_exec_type_ = PlannerManager::EdgeExecType::NONE;
                graph_inited_ = true;
            }
            if (!have_goal_){
                return;
            }
            changeFSMState(WAIT_GOAL, "FSM");
            break;
        }
        case WAIT_GOAL:
        {
            if (!have_goal_){
                return;
            }
            else{
                changeFSMState(PLAN_TRAJECTORY, "FSM");
            }
            break;
        }
        case PLAN_TRAJECTORY:
        {
            // set current pos as start pos
            start_pos_ = odom_pos_;
            // ------------------------------------------------------------
            // Case 1: current node is newly realized
            // Need to expand this node and choose the globally best next action
            // ------------------------------------------------------------
            if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::NONE) {

                planner_manager_->expandNode(start_pos_, goal_pos_, planner_manager_->current_node_id_);

                bool ok = planner_manager_->planGlobalBestAction(goal_pos_);
                if (!ok) {
                    ROS_WARN("[FSM]: No valid global action found.");
                    stopRobot();
                    return;
                }

                // If the selected action is an expand edge, we need to optimize a new trajectory
                if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::EXPAND_EDGE) {
                    bool ok2 = planner_manager_->prepareTrajectoryForCurrentEdge(start_pos_);
                    if (!ok2) {
                        ROS_WARN("[FSM]: Failed to prepare trajectory for current expand edge.");
                        stopRobot();
                        return;
                    }
                }
                // If the selected action is a graph path edge, do NOT replan trajectory here.
                // Later the EXEC state will directly use the cached bezier stored in the edge.
                else if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::PATH_EDGE) {
                    ROS_INFO("[FSM]: Selected action is a graph path edge, skip trajectory replanning.");
                }
                else {
                    ROS_WARN("[FSM]: current_edge_exec_type_ is NONE after planGlobalBestAction.");
                    stopRobot();
                    return;
                }

                changeFSMState(EXEC_TRAJECTORY, "FSM");
                break;
            }

            // ------------------------------------------------------------
            // Case 2: path is finished, and now we are ready to execute
            // the already-selected expand edge at the frontier node
            // ------------------------------------------------------------
            if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::EXPAND_EDGE) {
                bool ok = planner_manager_->prepareTrajectoryForCurrentEdge(start_pos_);
                if (!ok) {
                    ROS_WARN("[FSM]: Failed to prepare trajectory for expand edge.");
                    stopRobot();
                    return;
                }

                changeFSMState(EXEC_TRAJECTORY, "FSM");
                break;
            }

            // ------------------------------------------------------------
            // Case 3: PATH_EDGE should normally never enter PLAN_TRAJECTORY
            // ------------------------------------------------------------
            if (planner_manager_->current_edge_exec_type_ == PlannerManager::EdgeExecType::PATH_EDGE) {
                ROS_WARN("[FSM]: PATH_EDGE should not normally enter PLAN_TRAJECTORY.");
                stopRobot();
                return;
            }

            ROS_WARN("[FSM]: Unknown planning state in PLAN_TRAJECTORY.");
            stopRobot();
            return;
}
        case EXEC_TRAJECTORY:
        {
            // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TESTING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // check if reached the goal
            double distance_to_goal = (odom_pos_ - goal_pos_).norm();
            // ROS_INFO("[FSM]: Distance to goal: %.2f", distance_to_goal);
            if (distance_to_goal < 0.1){
                ROS_INFO("[FSM]: Reached the goal!");
                have_goal_ = false;
                stopRobot();
                changeFSMState(WAIT_GOAL, "FSM");
                return;
            }
        }
    }
}

void PlannerManagerFSM::printFSMCurrentState() {
    std::string state_str[4] = {"INIT", "WAIT_GOAL", "PLAN_TRAJECTORY", "EXEC_TRAJECTORY"};
    ROS_INFO("\033[36m[FSM]: Current state: %s\033[0m", state_str[current_state_].c_str());
}

void PlannerManagerFSM::changeFSMState(FSM_EXEC_STATE new_state, std::string pos_call) {
    std::string state_str[4] = {"INIT", "WAIT_GOAL", "PLAN_TRAJECTORY", "EXEC_TRAJECTORY"};
    int old_state = int(current_state_);
    current_state_ = new_state;
    ROS_INFO("\033[37m[FSM]: State: %s\033[0m", state_str[int(current_state_)].c_str());
}

void PlannerManagerFSM::quaternionToRPY(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
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

void PlannerManagerFSM::stopRobot() {
    ROS_INFO("[FSM]: Stop the robot!");
    geometry_msgs::Twist twist_cmd;
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    cmd_vel_pub_.publish(twist_cmd);
}

double PlannerManagerFSM::clampd(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

double PlannerManagerFSM::wrapToPi(double a) {
    a = std::fmod(a + M_PI, 2 * M_PI);
    if (a < 0)        a += 2 * M_PI;
    return a - M_PI;
}

double PlannerManagerFSM::projectToBezierSE2(const BezierSE2& traj, const Eigen::Vector2d& p, int M) {
    double best_t = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= M; ++i){
        double t = double(i) / double(M);
        const Eigen::Vector2d pt = traj.pos(t);
        const double d2 = (pt - p).squaredNorm();
        if (d2 < best_d2){
            best_d2 = d2;
            best_t = t;
        }
    }
    // local refinement
    double t0 = best_t;
    double step = 1.0 / double(M);
    for (int it = 0; it < 8; ++it) {
        double ta = clampd(t0 - step, 0.0, 1.0);
        double tb = t0;
        double tc = clampd(t0 + step, 0.0, 1.0);
        double da = (traj.pos(ta) - p).squaredNorm();
        double db = (traj.pos(tb) - p).squaredNorm();
        double dc = (traj.pos(tc) - p).squaredNorm();
        if (da < db && da < dc) t0 = ta;
        else if (dc < db && dc < da) t0 = tc;
        step *= 0.5;
    }
    return t0;
}

double PlannerManagerFSM::projectToBezierSE3(const BezierSE3& traj, const Eigen::Vector3d& p, int M) {
    double best_t = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= M; ++i){
        double t = double(i) / double(M);
        const Eigen::Vector3d pt = traj.pos(t);
        const double d2 = (pt - p).squaredNorm();
        if (d2 < best_d2){
            best_d2 = d2;
            best_t = t;
        }
    }
    double t0 = best_t;
    double step = 1.0 / double(M);
    for (int it = 0; it < 8; ++it) {
        double ta = clampd(t0 - step, 0.0, 1.0);
        double tb = t0;
        double tc = clampd(t0 + step, 0.0, 1.0);
        double da = (traj.pos(ta) - p).squaredNorm();
        double db = (traj.pos(tb) - p).squaredNorm();
        double dc = (traj.pos(tc) - p).squaredNorm();
        if (da < db && da < dc) t0 = ta;
        else if (dc < db && dc < da) t0 = tc;
        step *= 0.5;
    }
    return t0;
}

Eigen::Vector3d PlannerManagerFSM::so3Log(const Eigen::Matrix3d& R) {
    // Robust log map (small-angle safe)
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    const double theta = std::acos(cos_theta);

    Eigen::Vector3d w;
    if (theta < 1e-6) {
        // first-order approximation: vee(R - R^T)/2
        w << (R(2,1) - R(1,2)),
            (R(0,2) - R(2,0)),
            (R(1,0) - R(0,1));
        w *= 0.5;
        return w;
    }

    // w = theta/(2*sin(theta)) * vee(R - R^T)
    const double s = std::sin(theta);
    Eigen::Vector3d vee;
    vee << (R(2,1) - R(1,2)),
            (R(0,2) - R(2,0)),
            (R(1,0) - R(0,1));
    w = (theta / (2.0 * s)) * vee;
    return w;    
}

double PlannerManagerFSM::so3Angle(const Eigen::Matrix3d& R) {
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta);
}

double PlannerManagerFSM::so2Angle(const Eigen::Matrix2d& R) {
    return std::atan2(R(1,0), R(0,0));
}

void PlannerManagerFSM::publishGoalMarker() {
    // publish goal marker for visualization
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "odom";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "goal_marker";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_pos_[0];
    goal_marker.pose.position.y = goal_pos_[1];
    goal_marker.pose.position.z = goal_pos_[2];
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.pose.orientation.x = 0.0;   
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.scale.x = 0.2;
    goal_marker.scale.y = 0.2;
    goal_marker.scale.z = 0.2;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;

    goal_marker.lifetime = ros::Duration(0.0); // forever

    goal_marker_pub_.publish(goal_marker);
}

void PlannerManagerFSM::publishGlobalGraph() {
    visualization_msgs::Marker ray_list;
    ray_list.header.frame_id = "odom";
    ray_list.header.stamp = ros::Time::now();
    ray_list.ns = "global_graph";
    ray_list.id = 0;
    ray_list.type = visualization_msgs::Marker::LINE_LIST;
    ray_list.action = visualization_msgs::Marker::ADD;
    ray_list.scale.x = 0.02f;        // line width
    ray_list.color.r = 0.0f;         
    ray_list.color.g = 0.0f;
    ray_list.color.b = 1.0f;
    ray_list.color.a = 1.0f;

    geometry_msgs::Point p_start, p_end;
    for (size_t i = 0; i < planner_manager_->graph_points_for_visualization_.size(); i +=2){
        p_start.x = planner_manager_->graph_points_for_visualization_[i][0];
        p_start.y = planner_manager_->graph_points_for_visualization_[i][1];
        p_start.z = planner_manager_->graph_points_for_visualization_[i][2];
        p_end.x = planner_manager_->graph_points_for_visualization_[i+1][0];
        p_end.y = planner_manager_->graph_points_for_visualization_[i+1][1];
        p_end.z = planner_manager_->graph_points_for_visualization_[i+1][2];
        ray_list.points.push_back(p_start);
        ray_list.points.push_back(p_end);
    }
    global_graph_pub_.publish(ray_list);
}

void PlannerManagerFSM::visualizationCallback(const ros::TimerEvent &e) {
    if (have_goal_){
        publishGoalMarker();
    }
    if (!planner_manager_->graph_points_for_visualization_.empty()){
        publishGlobalGraph();
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("[FSM]: Plan manager node start");
    ros::init(argc, argv, "planner_manager_fsm");


    ros::NodeHandle nh("~");

    PlannerManagerFSM planner_manager_fsm;
    planner_manager_fsm.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO("[FSM node]: Plan manager loaded namespace %s", ns.c_str());

    ros::spin();
}