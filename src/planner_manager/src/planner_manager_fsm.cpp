#include "planner_manager_fsm.h"

void PlannerManagerFSM::init(ros::NodeHandle &nh) {
    node_ = nh;
    current_state_ = INIT;
    have_odom_ = false;
    have_goal_ = false;

    node_.param<int>("env_type", env_type_, 2); 
    ROS_INFO("[PlannerManagerFSM] env_type: %d", env_type_);
    std::vector<double> crop_size_vec;
    node_.param<std::vector<double>>("size_of_croped_pointcloud", crop_size_vec, {3, 3, 2});
    if (crop_size_vec.size() == 3) {
        size_of_croped_pointcloud_ = Eigen::Vector3d(crop_size_vec[0], crop_size_vec[1], crop_size_vec[2]);
    } else {
        size_of_croped_pointcloud_ = Eigen::Vector3d(3, 3, 2); // 默认值
        ROS_WARN("size_of_croped_pointcloud param size error, using default (3,3,2)");
    }

    planner_manager_.reset(new PlannerManager());
    planner_manager_->setEnvType(env_type_);
    planner_manager_->setSizeOfCropedPointcloud(size_of_croped_pointcloud_);
    planner_manager_->initPlannerModule(node_);

    /* Callback Function */
    FSM_timer_ = node_.createTimer(ros::Duration(0.2), &PlannerManagerFSM::FSMCallback, this);

    /* ROS subscribers */
    odom_sub_ = node_.subscribe("/odom", 1, &PlannerManagerFSM::odomCallback, this);
    goal_sub_ = node_.subscribe("/goal", 1, &PlannerManagerFSM::goalCallback, this);

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void PlannerManagerFSM::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    if (!msg) {
        ROS_WARN("Received null odometry message");
        return;
    }
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    odom_ori_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    quaternionToRPY(odom_ori_, odom_roll_, odom_pitch_, odom_yaw_);

    have_odom_ = true;
}

void PlannerManagerFSM::goalCallback(const geometry_msgs::PoseStampedPtr &msg) {
    if (!msg) {
        ROS_WARN("Received null goal message");
        return;
    }
    if (!have_odom_){
        return;
    }
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_ori_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    quaternionToRPY(goal_ori_, goal_roll_, goal_pitch_, goal_yaw_);

    stopRobot();
    ROS_INFO("[FSM]: Received new goal");
    changeFSMState(PLAN_TRAJECTORY, "FSM");
    have_goal_ = true;
}

void PlannerManagerFSM::FSMCallback(const ros::TimerEvent &e) {
    /* init phase wating for odom & goal */
    static int FSM_num = 0;
    FSM_num++;
    if (FSM_num == 2000){
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
        if (!have_odom_){
            return;
        }
        /* code */

        if (!have_goal_){
            return;
        }
        changeFSMState(WAIT_GOAL, "FSM");
        break;

    case WAIT_GOAL:
        if (!have_goal_){
            return;
        }
        else{
            changeFSMState(PLAN_TRAJECTORY, "FSM");
        }
        break;

    case PLAN_TRAJECTORY:
        /* code */
        break;
    }
}

void PlannerManagerFSM::printFSMCurrentState() {
    std::string state_str[3] = {"INIT", "WAIT_GOAL", "PLAN_TRAJECTORY"};
    ROS_INFO("\033[36m[FSM]: Current state: %s\033[0m", state_str[current_state_].c_str());
}

void PlannerManagerFSM::changeFSMState(FSM_EXEC_STATE new_state, std::string pos_call) {
    std::string state_str[3] = {"INIT", "WAIT_GOAL", "PLAN_TRAJECTORY"};
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