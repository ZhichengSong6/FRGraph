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

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <nav_msgs/Odometry.h>

struct RangeMap{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<std::vector<float>> azimuth;
    std::vector<std::vector<float>> elevation;
    std::vector<std::vector<float>> range;
};

enum class EdgeClass{FF, FU}; // FF: finite-finite, FU: finite-unlimited
enum class HEdgeType{NONE, L, R}; // L: near obstacle on the left, R: near obstacle on the right
enum class VEdgeType{NONE, U, D}; // U: near obstacle on the up, D: near obstacle on the down

struct Edge{
    int v, u, type; // type: 0-horizontal edge, 1-vertical edge
    EdgeClass edge_class;
    HEdgeType h_edge_type;
    VEdgeType v_edge_type;
};

class GapExtractor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<GapExtractor> Ptr;

    GapExtractor() {}
    ~GapExtractor() {}

    void initialize(ros::NodeHandle &nh, bool env_type);

    void pointCloudToRangeMap();
    void medianFilter();
    void fillTinyHoles();
    void detectEdges();

    /* Helper functions */
    inline float angDist(float th1, float ph1, float th2, float ph2);
    void fetchRangeForCompare(int v, int u, float r_max, float& r, bool& is_unknown);

private:
    ros::NodeHandle node_;
    
    bool env_type_; // 0 for 2D, 1 for 3D environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    
    // range map parameters
    int range_map_width_, range_map_height_;
    RangeMap range_map_;
    
    // get tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_odom_;
    tf2_ros::Buffer tf_buffer_odom_;
    
    geometry_msgs::TransformStamped::Ptr lidar_to_base_ptr_, base_to_odom_ptr_;
    Eigen::Matrix4f T_lidar_base_mat_ = Eigen::Matrix4f::Identity();
    
    /* Callbacks */
    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void odomCallback(const ros::TimerEvent &e);
    void visualizationCallback(const ros::TimerEvent &e);
    void extractGapCallback(const ros::TimerEvent &e);

    /* Publish topics */
    void publishRangeMapAsImage();
    
    /* Subscribers */
    ros::Subscriber velodyne_sub_;

    /* Publishers */
    ros::Publisher image_pub_;

    /* Timers */
    ros::Timer gap_extractor_timer_;
    ros::Timer visualization_timer_;
    ros::Timer odom_timer_;
};

#endif // GAP_EXTRACTOR_H