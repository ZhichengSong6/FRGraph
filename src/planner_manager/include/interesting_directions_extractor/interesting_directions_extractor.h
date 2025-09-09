#ifndef _INTERESTING_DIRECTION_EXTRACTOR_H_
#define _INTERESTING_DIRECTION_EXTRACTOR_H_

#include <Eigen/Dense>
#include <vector>

#include <ros/ros.h>

#include <decomp_ros_utils/data_ros_utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

struct EdgePoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
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
    void getInfoOfEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    private:
    ros::NodeHandle node_;
    
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

    /* Callbacks */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg);

    /* Timer */
    ros::Timer direction_extraction_timer_;

    /* Publisher */
    ros::Publisher interesting_pts_pub_;

    /* Subscriber */
    ros::Subscriber velodyne_sub_;   // pointcloud for 3D environment
    ros::Subscriber scan2d_sub_;     // laser scan for 2D environment
};

#endif  // _INTERESTING_DIRECTION_EXTRACTOR_H_