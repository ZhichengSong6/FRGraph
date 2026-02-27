#include "planner_manager_fsm.h"

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

void PlannerManagerFSM::publishCmdCallback(const ros::TimerEvent &e) {
    // publish cmd vel
    if (current_state_ == INIT || current_state_ == WAIT_GOAL){
        return;
    }
    if (planner_manager_->trajectory_points_temp_.empty()){
        ROS_WARN_THROTTLE(2.0, "[FSM]: No trajectory points to follow!");
        return;
    }
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TESTING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // based on trajectory_points_temp_ and current odom_pos_, publish cmd_vel
    int traj_point_index = 0;
    double min_distance = 1000;
    for (int i = 1; i < planner_manager_->trajectory_points_temp_.size(); ++i) {
        double distance = (planner_manager_->trajectory_points_temp_[i] - odom_pos_).norm();
        if (distance < min_distance) {
            min_distance = distance;
            traj_point_index = i;
        }
    }
    geometry_msgs::Twist twist_cmd;
    if (traj_point_index + 2 > planner_manager_->trajectory_points_temp_.size()){
        twist_cmd.linear.x = (planner_manager_->trajectory_points_temp_.back()[0] - odom_pos_[0]) * 1.5 + (-odom_vel_[0]) * 0.5;
        twist_cmd.linear.y = (planner_manager_->trajectory_points_temp_.back()[1] - odom_pos_[1]) * 1.5 + (-odom_vel_[1]) * 0.5;
        twist_cmd.linear.z = (planner_manager_->trajectory_points_temp_.back()[2] - odom_pos_[2]) * 1.5 + (-odom_vel_[2]) * 0.5;
        twist_cmd.angular.x = 0.0;
        twist_cmd.angular.y = 0.0;
        twist_cmd.angular.z = 0.0;
    }else{
        twist_cmd.linear.x = (planner_manager_->trajectory_points_temp_[traj_point_index + 1][0] - odom_pos_[0]) * 1.5 + (-odom_vel_[0]) * 0.5;
        twist_cmd.linear.y = (planner_manager_->trajectory_points_temp_[traj_point_index + 1][1] - odom_pos_[1]) * 1.5 + (-odom_vel_[1]) * 0.5;
        twist_cmd.linear.z = (planner_manager_->trajectory_points_temp_[traj_point_index + 1][2] - odom_pos_[2]) * 1.5 + (-odom_vel_[2]) * 0.5;
        twist_cmd.angular.x = 0.0;
        twist_cmd.angular.y = 0.0;
        twist_cmd.angular.z = 0.0;
    }
    // currently thecmd_vel is in odom frame, need to transform it to base_link frame
    Eigen::Vector3d vel_odom(twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z);
    Eigen::Matrix3d R_odom_base = odom_ori_.toRotationMatrix().transpose();
    Eigen::Vector3d vel_base = R_odom_base * vel_odom;
    twist_cmd.linear.x = vel_base[0];;
    twist_cmd.linear.y = vel_base[1];
    twist_cmd.linear.z = vel_base[2];
// ROS_INFO("[FSM]: Publishing cmd_vel: linear=(%.2f, %.2f, %.2f)", twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z);
    cmd_vel_pub_.publish(twist_cmd);
}

void PlannerManagerFSM::replanCheckCallback(const ros::TimerEvent &e) {
    // a callback to check if need to replan
    // A: achieved subgoal
    // B: no more gap above (TBD)
    if (current_state_ != EXEC_TRAJECTORY){
        return;
    }
    double distance_to_subgoal = (odom_pos_ - planner_manager_->current_node_->replan_pos_).norm();
    // if (distance_to_subgoal < 0.1){
    //     ROS_INFO("[FSM]: Reached subgoal, replanning...");
    //     changeFSMState(PLAN_TRAJECTORY, "replanCheckCallback");
    //     return;
    // }
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
            Eigen::Vector3d start_pos = odom_pos_;
            planner_manager_->graph_points_for_visualization_.clear();
            planner_manager_->free_regions_graph_ptr_->setRootNode(start_pos);
            planner_manager_->current_node_ = planner_manager_->free_regions_graph_ptr_->getRootNode();

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
            planner_manager_->planTrajectory(start_pos_, goal_pos_, planner_manager_->current_node_);
            changeFSMState(EXEC_TRAJECTORY, "FSM");
            break;
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