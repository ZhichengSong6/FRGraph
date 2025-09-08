#include "interesting_directions_extractor/interesting_directions_extractor.h"

void InterestingDirectionExtractor::initialize(ros::NodeHandle &nh, bool env_type) {
    node_ = nh;
    env_type_ = env_type;

    node_.param<int>("direction_extractor/num_of_corner_pts_", num_of_corner_pts_, 5); // default 5
    ROS_INFO("Number of corner points to extract: %d", num_of_corner_pts_);

    // set empty point cloud
    pointcloud_croped_base_frame_.clear();
    pointcloud_croped_base_frame_2d_.clear();

    if (env_type_) {
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections3D, this);
    } else {
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections2D, this);
    }
}

void InterestingDirectionExtractor::extractInterestingDirections2D(const ros::TimerEvent &e) {
    // Implementation of the direction extraction logic
    if (pointcloud_croped_base_frame_2d_.empty()) {
        return;
    }

    interesting_pts_2d_.clear();
    // convert vec_Vec2f to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : pointcloud_croped_base_frame_2d_) {
        cloud->points.push_back(pcl::PointXYZ(pt[0], pt[1], 0.0));
    }
    
    extractCornerPoints(cloud, interesting_pts_2d_, num_of_corner_pts_);
}

void InterestingDirectionExtractor::extractInterestingDirections3D(const ros::TimerEvent &e) {
    // Implementation of the direction extraction logic
    if (pointcloud_croped_base_frame_.empty()) {
        return;
    }
}

void InterestingDirectionExtractor::extractCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<Eigen::Vector2d>& corner_points, int num_corners) {
    if (cloud->points.size() ==0){
        return;
    }
    curvatures_.clear();
    cloud_neighbor_picked_.clear();
    curvature_sorted_indices_.clear();
    computeCurvature(cloud, curvatures_);
}

void InterestingDirectionExtractor::computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<double>& curvatures) {
    if (cloud->points.size() < 5) {
        ROS_WARN("Not enough points to compute curvature");
        return;
    }

    for (int i = 5; i < cloud->points.size() - 5; ++i) {
        float diffX  = cloud->points[i-5].x + cloud->points[i-4].x + cloud->points[i-3].x + cloud->points[i-2].x + cloud->points[i-1].x - 10 * cloud->points[i].x
                     + cloud->points[i+5].x + cloud->points[i+4].x + cloud->points[i+3].x + cloud->points[i+2].x + cloud->points[i+1].x;
        float diffY  = cloud->points[i-5].y + cloud->points[i-4].y + cloud->points[i-3].y + cloud->points[i-2].y + cloud->points[i-1].y - 10 * cloud->points[i].y
                     + cloud->points[i+5].y + cloud->points[i+4].y + cloud->points[i+3].y + cloud->points[i+2].y + cloud->points[i+1].y;
        float diffZ  = cloud->points[i-5].z + cloud->points[i-4].z + cloud->points[i-3].z + cloud->points[i-2].z + cloud->points[i-1].z - 10 * cloud->points[i].z
                     + cloud->points[i+5].z + cloud->points[i+4].z + cloud->points[i+3].z + cloud->points[i+2].z + cloud->points[i+1].z;
        curvatures[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }

    // get the curvature of the first 5 and last 5 points
    for (int i = 0; i < 5; ++i) {
        float diffX = cloud->points[i].x + cloud->points[i+1].x + cloud->points[i+2].x + cloud->points[i+3].x + cloud->points[i+4].x - 10 * cloud->points[i].x;
        float diffY = cloud->points[i].y + cloud->points[i+1].y + cloud->points[i+2].y + cloud->points[i+3].y + cloud->points[i+4].y - 10 * cloud->points[i].y;
        float diffZ = cloud->points[i].z + cloud->points[i+1].z + cloud->points[i+2].z + cloud->points[i+3].z + cloud->points[i+4].z - 10 * cloud->points[i].z;
        
        // base on the index of the point, add certain points from the end of the cloud to make sure 10 points are used
        for (int j = 0; j < 5 - i; ++j) {
            diffX += cloud->points[cloud->points.size() - 1 - j].x;
            diffY += cloud->points[cloud->points.size() - 1 - j].y;
            diffZ += cloud->points[cloud->points.size() - 1 - j].z;
        }
        for (int j = 0; j < i; ++j) {
            diffX += cloud->points[i + 1 + j].x;
            diffY += cloud->points[i + 1 + j].y;
            diffZ += cloud->points[i + 1 + j].z;
        }
        curvatures[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }


}