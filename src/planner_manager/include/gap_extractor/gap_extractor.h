#ifndef GAP_EXTRACTOR_H
#define GAP_EXTRACTOR_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>

class GapExtractor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GapExtractor();
    ~GapExtractor();

    void initialize(ros::NodeHandle &nh, bool env_type);

    void pointCloudToRangeMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::vector<float>>& range_map, float grid_size, int map_width, int map_height);

private:
    ros::NodeHandle nh_;

    bool env_type_; // 0 for 2D, 1 for 3D environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    std::vector<std::vector<float>> range_map_;
    pcl::RangeImageSpherical range_image_;

    /* Callbacks */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void visualizationCallback(const ros::TimerEvent &e);

    /* Subscribers */
    ros::Subscriber velodyne_sub_;
};

#endif // GAP_EXTRACTOR_H