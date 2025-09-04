#ifndef PLANNER_MANAGER_FSM
#define PLANNER_MANAGER_FSM

#include <ros/ros.h>

#include <Eigen/Dense>

#include "planner_manager.h"

class PlannerManagerFSM {
    private:
    enum FSM_EXEC_STATE {
        INIT,
        WAIT_GOAL
    };
    FSM_EXEC_STATE current_state_;
    int env_type_;
    Eigen::Vector3d size_of_croped_pointcloud_;

    /* ROS utils */
    ros::NodeHandle node_;
    
    PlannerManager::Ptr planner_manager_;

    public:
    PlannerManagerFSM(/* args */) {}
    ~PlannerManagerFSM() {}


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // PLANNER_MANAGER_FSM