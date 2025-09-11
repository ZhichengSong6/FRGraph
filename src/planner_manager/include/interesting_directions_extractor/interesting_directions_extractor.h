#ifndef _INTERESTING_DIRECTION_EXTRACTOR_H_
#define _INTERESTING_DIRECTION_EXTRACTOR_H_

#include <Eigen/Dense>
#include <vector>
#include <numeric>

#include <ros/ros.h>

#include <decomp_ros_utils/data_ros_utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

struct EdgePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    int index_in_cloud;
    bool left_obstacle; // true if obstacle on the left side
    bool right_obstacle; // true if obstacle on the right side
};

class InterestingDirectionExtractor {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::unique_ptr<InterestingDirectionExtractor> Ptr;

    InterestingDirectionExtractor() {}
    ~InterestingDirectionExtractor() {}

    std::vector<Eigen::Vector2d> interesting_pts_2d_;
    std::vector<Eigen::Vector3d> interesting_pts_3d_;
    
    void initialize(ros::NodeHandle &nh, bool env_type);
    void setSizeOfCroppedPointcloud(const Eigen::Vector3d &size) { size_of_cropped_pointcloud_ = size; }
    
    void extractInterestingDirections2D(const ros::TimerEvent &e);
    void extractInterestingDirections3D(const ros::TimerEvent &e);
    
    void extractEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_edges);
    
    void computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void sortIndicesBasedOnCurvature();
    void selectEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_points);
    void sortEdgePoints();
    void getInfoOfEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void checkGoalPose2D();

    void ExtractDirectionsFromEdgePoints();
    
    void publishEdgePoints();
    void publishInterestingDirections2D();

    private:
    ros::NodeHandle node_;

    Eigen::Vector3d goal_pos_;
    Eigen::Quaterniond goal_ori_;
    double goal_roll_, goal_pitch_, goal_yaw_;
    Eigen::Vector3d odom_pos_;
    Eigen::Quaterniond odom_ori_;
    double odom_roll_, odom_pitch_, odom_yaw_;
    
    bool env_type_; // 0 for 2D, 1 for 3D environment
    int num_of_edge_pts_;
    double curvature_threshold_;
    int num_of_neighbors_to_cluster_;
    double clustering_distance_threshold_; // meters
    Eigen::Vector3d size_of_cropped_pointcloud_;

    vec_Vec3f pointcloud_cropped_scan_frame_;
    vec_Vec2f pointcloud_cropped_scan_frame_2d_;

    std::vector<double> curvatures_;
    std::vector<int> cloud_neighbor_picked_;
    std::vector<int> curvature_sorted_indices_;
    std::vector<Eigen::Vector3d> edge_points_;
    std::vector<int> edge_points_indices_in_cloud_;
    std::vector<EdgePoint> edge_points_with_info_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_odom_;
    tf2_ros::Buffer tfBuffer_odom_;

    geometry_msgs::TransformStamped::Ptr lidar_to_base_ptr_, base_to_odom_ptr_;
    Eigen::Matrix4f T_lidar_base_mat_ = Eigen::Matrix4f::Identity();

    /* Callbacks */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void odomTimerCallback(const ros::TimerEvent &e);
    void visualizationCallback(const ros::TimerEvent &e);

    /* Timer */
    ros::Timer direction_extraction_timer_;
    ros::Timer visualization_timer_;
    ros::Timer odom_timer_;

    /* Publisher */
    ros::Publisher edge_pts_pub_;       // for debug
    ros::Publisher direction_2d_pub_;   // interesting directions in 2D

    /* Subscriber */
    ros::Subscriber velodyne_sub_;   // pointcloud for 3D environment
    ros::Subscriber scan2d_sub_;     // laser scan for 2D environment
    ros::Subscriber goal_sub_;

    /* Helper Function */
    void quaternionToRPY(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw);
};

#endif  // _INTERESTING_DIRECTION_EXTRACTOR_H_