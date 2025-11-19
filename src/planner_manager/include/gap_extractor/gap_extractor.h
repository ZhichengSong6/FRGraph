#ifndef GAP_EXTRACTOR_H
#define GAP_EXTRACTOR_H

#include <ros/ros.h>

#include <Eigen/Dense>
#include <vector>

#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>

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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
    double r;
    EdgeClass edge_class;
    HEdgeType h_edge_type;
    VEdgeType v_edge_type;
};

struct GapMasks{
    std::vector<std::vector<uint8_t>> open;
    std::vector<std::vector<uint8_t>> limited;
    std::vector<std::vector<uint8_t>> free; // when no obstacle detected (each layer)
};

struct EdgeParameters{
    float a_h = 0.20f; // horizontal edge detection parameter
    float b_h = 0.02f;
    float lambda_h = 0.5f;
    float eps_h = 1e-3f;
    float a_v = 0.30f; // vertical edge detection parameter
    float b_v = 0.05f;
    float lambda_v = 0.5f;
    float eps_v = 2e-3f;
};

struct GapRegion{
    std::vector<std::pair<int,int>> pixels; // (v,u) pixel coordinates
    int size = 0;
    int v_min = std::numeric_limits<int>::max();
    int v_max = std::numeric_limits<int>::min();

    // Spherical mean direction of pixels in this gap region
    float dir_x = 0.f;
    float dir_y = 0.f;
    float dir_z = 0.f;
    float center_yaw = 0.f;
    float center_elev = 0.f;

    // Angular extents of the component
    float yaw_span = 0.f;
    float elev_span = 0.f;
};

struct GapSubRegion{
    std::vector<std::pair<int,int>> pixels; // (v,u) pixel coordinates
    int size = 0;
    int v_min = std::numeric_limits<int>::max();
    int v_max = std::numeric_limits<int>::min();
    
    // representative direction
    float center_yaw = 0.f;
    float center_elev = 0.f;

    // representative pixel on the grid
    float yaw_span = 0.f;
    float elev_span = 0.f;
    float range_mean = 0.f;
};

struct Parameters{
    // gap extraction parameters
    int min_pixels_in_open_gap_region = 30;
    float yaw_split_threshold = M_PI / 4; // 45 degrees
    float elev_split_threshold = M_PI / 9; // 20 degrees
    float open_gap_yaw_span = M_PI / 4; // 45 degrees
    float open_gap_elev_span = M_PI / 9; // 20 degrees
    int min_pixels_in_subregion = 40;
    int range_map_width = 1600;  // lidar horizontal resolution
    int range_map_height = 32;  // lidar vertical resolution
    int map_size = 5; // meters

    float yaw_split_threshold_in_limited_gap_region = M_PI / 6; // 30 degrees
    float elev_split_threshold_in_limited_gap_region = M_PI / 9; // 20 degrees
    int min_pixels_in_limited_gap_region = 24;

    float limited_gap_yaw_span = M_PI / 6; // 30 degrees
    float limited_gap_elev_span = M_PI / 6; // 30 degrees
    float min_pixels_in_limited_subregion = 32;
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
    void buildGapMasks();
    void buildGapMasks_FromSingleFFEdge(std::vector<std::vector<uint8_t>>& mask_limited);
    void extractGapRegions();
    void bfsGapRegion(int v0, int u0, const std::vector<std::vector<uint8_t>>& mask, std::vector<std::vector<uint8_t>>& visited, GapRegion& region);
    void splitAllGapRegions();
    void splitOpenGapRegion(const GapRegion& region, float yaw_sub_span, float elev_sub_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions);
    void collectShiftedAnglesAndBBox(const GapRegion& region, float yaw_offset,
                                    std::vector<float>& yaw_s, std::vector<float>& elev_s,
                                    float& yaw_min, float& yaw_max,
                                    float& elev_min, float& elev_max) const;
    void assignPixelsToSubregions(const GapRegion& region, 
                                  const std::vector<float>& yaw_s, const std::vector<float>& elev_s,
                                  float center_yaw_shifted, float center_elev,
                                  float yaw_threshold, float elev_threshold,
                                  int Ny, int Ne,
                                  std::vector<std::vector<GapSubRegion>>& subregions) const;
    void mergeSmallSubregions(std::vector<std::vector<GapSubRegion>>& subregions, int Ny, int Ne, int min_pixels) const;
    void computeStateForCell(GapSubRegion& cell) const;
    void computeStateForSubregions(std::vector<std::vector<GapSubRegion>>& subregions) const;
    void splitLimitedGapRegion(const GapRegion& region, float yaw_sub_span, float elev_sub_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions);
    void splitFreeGapRegion(const GapRegion& region, float yaw_sub_span, float elev_sub_span, int min_pixels, std::vector<std::vector<GapSubRegion>>& subregions);

    /* Helper functions */
    inline float angDist(float th1, float ph1, float th2, float ph2);
    void fetchRangeForCompare(int v, int u, float& r, bool& is_free);

private:
    ros::NodeHandle node_;
    
    bool env_type_; // 0 for 2D, 1 for 3D environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    
    // range map parameters
    int range_map_width_, range_map_height_;
    int v_margin_ = 0;
    Parameters params_;
    RangeMap range_map_;
    std::vector<Edge> selected_edges_;
    GapMasks gap_masks_;
    EdgeParameters edge_params_;
    std::vector<GapRegion> gap_regions_open_;
    std::vector<GapRegion> gap_regions_limited_;
    std::vector<GapRegion> gap_regions_free_;
    std::vector<std::vector<std::vector<GapSubRegion>>> open_gap_subregions_;
    std::vector<std::vector<std::vector<GapSubRegion>>> limited_gap_subregions_;
    std::vector<std::vector<std::vector<GapSubRegion>>> free_gap_subregions_;

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
    void publishEdges();
    visualization_msgs::Marker maskToMarkerPoints(const std::vector<std::vector<uint8_t>>& mask, 
                                const RangeMap& range_map, float radius,
                            float r, float g, float b, int id);
    void publishMasks();
    void publishSubGapRegions();

    /* Subscribers */
    ros::Subscriber velodyne_sub_;

    /* Publishers */
    ros::Publisher image_pub_;
    ros::Publisher edge_pub_;
    ros::Publisher mask_pub_;
    ros::Publisher subregion_pub_;

    /* Timers */
    ros::Timer gap_extractor_timer_;
    ros::Timer visualization_timer_;
    ros::Timer odom_timer_;
};

#endif // GAP_EXTRACTOR_H