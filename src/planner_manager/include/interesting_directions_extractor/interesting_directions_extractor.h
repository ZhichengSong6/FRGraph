#ifndef _INTERESTING_DIRECTION_EXTRACTOR_H_
#define _INTERESTING_DIRECTION_EXTRACTOR_H_

#include <Eigen/Dense>
#include <vector>

#include <ros/ros.h>

#include <decomp_ros_utils/data_ros_utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class InterestingDirectionExtractor {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::unique_ptr<InterestingDirectionExtractor> Ptr;

    InterestingDirectionExtractor() {}
    ~InterestingDirectionExtractor() {}

    int num_of_corner_pts_;
    std::vector<Eigen::Vector2d> interesting_pts_2d_;
    std::vector<Eigen::Vector3d> interesting_pts_3d_;

    void initialize(ros::NodeHandle &nh, bool env_type);
    void setPointCloud(const vec_Vec3f &pointcloud) { pointcloud_croped_base_frame_ = pointcloud; }
    void setPointCloud(const vec_Vec2f &pointcloud_2d) { pointcloud_croped_base_frame_2d_ = pointcloud_2d; }
    
    void extractInterestingDirections2D(const ros::TimerEvent &e);
    void extractInterestingDirections3D(const ros::TimerEvent &e);

    void extractCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<Eigen::Vector3d>& corner_points, int num_corners);
    void extractCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<Eigen::Vector2d>& corner_points, int num_corners);

    void computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<double>& curvatures);

    private:
    ros::NodeHandle node_;

    bool env_type_; // 0 for 2D, 1 for 3D environment

    vec_Vec3f pointcloud_croped_base_frame_;
    vec_Vec2f pointcloud_croped_base_frame_2d_;

    std::vector<double> curvatures_;
    std::vector<int> cloud_neighbor_picked_;
    std::vector<int> curvature_sorted_indices_;

    /* Timer */
    ros::Timer direction_extraction_timer_;

    /* Publisher */
    ros::Publisher interesting_pts_pub_;
};

#endif  // _INTERESTING_DIRECTION_EXTRACTOR_H_