#include "gap_extractor/gap_extractor.h"
 
 void GapExtractor::initialize(ros::NodeHandle &nh, bool env_type)
 {
    nh_ = nh;
    env_type_ = env_type;

    velodyne_sub_ = nh_.subscribe("velodyne_points", 1, &GapExtractor::velodyneCallback, this);
 }

 void GapExtractor::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
 {
    // Process the Velodyne point cloud data
    if (msg->data.size() == 0)
    {
        ROS_WARN("[GapExtractor] Received empty point cloud data");
        return;
    }
    // process point cloud to pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcl::fromROSMsg(*msg, *cloud_ptr_);
 }

 void GapExtractor::pointCloudToRangeMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::vector<float>>& range_map, float grid_size, int map_width, int map_height)
 {
   // Convert point cloud to range map
   // Initialize range map with max float values
   range_map.clear();
   range_map.resize(map_height, std::vector<float>(map_width, std::numeric_limits<float>::max()));

   // 计算中心点坐标（像素坐标）
   int cx = map_width / 2;
   int cy = map_height / 2;

   for (const auto& pt : cloud->points) {
      // 只考虑z在一定范围内的点（可选）
      // if (pt.z < -1.0 || pt.z > 1.0) continue;

      // 计算点在grid上的索引
      int x_idx = static_cast<int>(std::round(pt.x / grid_size)) + cx;
      int y_idx = static_cast<int>(std::round(pt.y / grid_size)) + cy;

      if (x_idx >= 0 && x_idx < map_width && y_idx >= 0 && y_idx < map_height) {
         float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
         // 只保留最近的点
         if (range < range_map[y_idx][x_idx]) {
            range_map[y_idx][x_idx] = range;
         }
      }
   }
   // 现在range_map[y][x]里存的是每个cell的最小距离（未命中为max float）
 }