#include "planner_manager_fsm.h"

void PlannerManagerFSM::init(ros::NodeHandle &nh) {
    node_ = nh;
    current_state_ = INIT;

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