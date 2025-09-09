#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Dense>

#include <decomp_ros_utils/data_ros_utils.h>

#include "free_regions_graph/free_regions_graph.h"
#include "interesting_directions_extractor/interesting_directions_extractor.h"

class PlannerManager {
    private:
    ros::NodeHandle node_;
    bool env_type_;
    Eigen::Vector3d size_of_cropped_pointcloud_;

    public:
    PlannerManager() {}
    ~PlannerManager() {}
    void setEnvType(int env_type) { env_type_ = env_type; }
    void setSizeOfCroppedPointcloud(const Eigen::Vector3d &size) { size_of_cropped_pointcloud_ = size; }

    typedef std::unique_ptr<PlannerManager> Ptr;

    /* ROS Subscriber */
    ros::Subscriber velodyne_sub_;   // pointcloud for 3D environment
    ros::Subscriber scan2d_sub_;     // laser scan for 2D environment

    /* ROS Publisher */

    void initPlannerModule(ros::NodeHandle &nh);

    /* Callback Functions */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void odomTimerCallback(const ros::TimerEvent &event);

    ros::Timer odom_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_odom_;
    tf2_ros::Buffer tfBuffer_odom_;

    geometry_msgs::TransformStamped::Ptr lidar_to_base_ptr_, base_to_odom_ptr_;
    Eigen::Matrix4f T_lidar_base_mat = Eigen::Matrix4f::Identity();

    vec_Vec3f pointcloud_cropped_odom_frame_;
    vec_Vec2f pointcloud_cropped_odom_frame_2d_;

    FreeRegionsGraph::Ptr free_regions_graph_ptr_;
    GraphNode *current_node_;

    InterestingDirectionExtractor::Ptr interesting_direction_extractor_ptr_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // PLANNER_MANAGER_H