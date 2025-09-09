#ifndef PLANNER_MANAGER_FSM
#define PLANNER_MANAGER_FSM

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>

#include "planner_manager.h"

static ros::Publisher cmd_vel;

class PlannerManagerFSM {
    private:
    enum FSM_EXEC_STATE {
        INIT,
        WAIT_GOAL,
        PLAN_TRAJECTORY,
    };
    FSM_EXEC_STATE current_state_;

    PlannerManager::Ptr planner_manager_;

    int env_type_;

    bool have_odom_, have_goal_;
    Eigen::Vector3d size_of_cropped_pointcloud_;

    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_ori_;
    double odom_roll_, odom_pitch_, odom_yaw_;

    Eigen::Vector3d goal_pos_;
    Eigen::Quaterniond goal_ori_;
    double goal_roll_, goal_pitch_, goal_yaw_;

    /* ROS utils */
    ros::NodeHandle node_;

    /* ROS publishers */
    ros::Publisher cmd_vel_pub_;

    /* ROS subscribers */
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;

    ros::Timer FSM_timer_;

    /* callback functions */
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void FSMCallback(const ros::TimerEvent &e);

    /* FSM function */
    void changeFSMState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMCurrentState();

    /* Helper functions */
    void quaternionToRPY(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw);
    void stopRobot();


    public:
    PlannerManagerFSM(/* args */) {}
    ~PlannerManagerFSM() {}


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // PLANNER_MANAGER_FSM