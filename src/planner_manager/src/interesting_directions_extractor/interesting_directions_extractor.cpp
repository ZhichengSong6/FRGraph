#include "interesting_directions_extractor/interesting_directions_extractor.h"

void InterestingDirectionExtractor::initialize(ros::NodeHandle &nh, bool env_type) {
    node_ = nh;
    env_type_ = env_type;

    node_.param<int>("direction_extractor/num_of_edge_pts_", num_of_edge_pts_, 5); // default 5
    ROS_INFO("Number of edge points to extract: %d", num_of_edge_pts_);
    node_.param<double>("direction_extractor/curvature_threshold_", curvature_threshold_, 0.8); // default 0.8
    ROS_INFO("Curvature threshold: %f", curvature_threshold_);
    node_.param<int>("direction_extractor/num_of_neighbors_to_cluster_", num_of_neighbors_to_cluster_, 10); // default 5
    ROS_INFO("Number of neighbors to cluster: %d", num_of_neighbors_to_cluster_);
    node_.param<double>("direction_extractor/clustering_distance_threshold_", clustering_distance_threshold_, 0.05); // default 0.05
    ROS_INFO("Clustering distance threshold: %f", clustering_distance_threshold_);
   
    // set empty point cloud
    pointcloud_cropped_scan_frame_.clear();
    pointcloud_cropped_scan_frame_2d_.clear();

    if (env_type_) {
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &InterestingDirectionExtractor::velodyneCallback, this);
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections3D, this);
    } else {
        scan2d_sub_ = node_.subscribe("/scan", 1, &InterestingDirectionExtractor::scan2dCallback, this);
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections2D, this);
    }
}

void InterestingDirectionExtractor::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Process the Velodyne point cloud data
    if (msg->data.size() == 0)
        {
        ROS_WARN("[InterestingDirectionExtractor] Received empty point cloud data");
        return;
        }
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

    vec_Vec3f pointcloud = DecompROS::cloud_to_vec(cloud);
    vec_Vec3f pointcloud_cropped;

    // crop the point cloud
    for (const auto &pt : pointcloud) {
        if (pt[0] > -size_of_cropped_pointcloud_[0]/2 && pt[0] < size_of_cropped_pointcloud_[0]/2 &&
            pt[1] > -size_of_cropped_pointcloud_[1]/2 && pt[1] < size_of_cropped_pointcloud_[1]/2 &&
            pt[2] > -size_of_cropped_pointcloud_[2]/2 && pt[2] < size_of_cropped_pointcloud_[2]/2) {
                pointcloud_cropped.push_back(pt);
            }
    }

    // pointcloud are in scan frame
    pointcloud_cropped_scan_frame_ = pointcloud_cropped;
}

void InterestingDirectionExtractor::scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    // Process the 2D laser scan data
    if (msg->ranges.size() == 0)
        {
        ROS_WARN("[InterestingDirectionExtractor] Received empty laser scan data");
        return;
        }
    vec_Vec2f pointcloud_2d;
    double angle = msg->angle_min;
    for (const auto &range : msg->ranges) {
        if (range < msg->range_max && range > msg->range_min) {
            double x = range * cos(angle);
            double y = range * sin(angle);
            pointcloud_2d.push_back(Eigen::Vector2d(x, y));
        }
        angle += msg->angle_increment;
    }

    vec_Vec2f pointcloud_cropped_2d;

    // crop the point cloud
    for (const auto &pt : pointcloud_2d) {
        if (pt[0] > -size_of_cropped_pointcloud_[0]/2 && pt[0] < size_of_cropped_pointcloud_[0]/2 &&
            pt[1] > -size_of_cropped_pointcloud_[1]/2 && pt[1] < size_of_cropped_pointcloud_[1]/2) {
                pointcloud_cropped_2d.push_back(pt);
            }
    }
    // pointcloud are in scan frame
    pointcloud_cropped_scan_frame_2d_ = pointcloud_cropped_2d;
}

void InterestingDirectionExtractor::extractInterestingDirections2D(const ros::TimerEvent &e) {
    // Implementation of the direction extraction logic
    if (pointcloud_cropped_scan_frame_2d_.empty()) {
        return;
    }

    interesting_pts_2d_.clear();
    edge_points_with_info_.clear();
    // convert vec_Vec2f to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : pointcloud_cropped_scan_frame_2d_) {
        cloud->points.push_back(pcl::PointXYZ(pt[0], pt[1], 0.0));
    }

    extractEdgePoints(cloud, num_of_edge_pts_);
}

void InterestingDirectionExtractor::extractInterestingDirections3D(const ros::TimerEvent &e) {
    // Implementation of the direction extraction logic
    if (pointcloud_cropped_scan_frame_.empty()) {
        return;
    }
}

void InterestingDirectionExtractor::extractEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_edges) {
    if (cloud->points.size() ==0){
        return;
    }
    curvatures_.clear();
    cloud_neighbor_picked_.clear();
    curvature_sorted_indices_.clear();
    edge_points_.clear();
    edge_points_indices_in_cloud_.clear();
    computeCurvature(cloud);
    sortIndicesBasedOnCurvature();
    selectEdgePoints(cloud, num_edges);
}

void InterestingDirectionExtractor::computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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
        curvatures_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
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
        curvatures_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }
}

void InterestingDirectionExtractor::sortIndicesBasedOnCurvature() {
    // sort cloud_sort_ind_ based on curvatures
    std::sort(curvature_sorted_indices_.begin(), curvature_sorted_indices_.end(), [&](const int &a, const int &b) {
        return curvatures_[a] > curvatures_[b];
    });
}

void InterestingDirectionExtractor::selectEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_points) {
    int count = 0;
    for (int k = 0; k < curvature_sorted_indices_.size(); ++k) {
        int index = curvature_sorted_indices_[k];
        if (cloud_neighbor_picked_[index] == 0 && curvatures_[index] > curvature_threshold_) {
            count++;
            if (count < num_points){
                edge_points_.push_back(Eigen::Vector3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z));
            } else {
                break;
            }
            cloud_neighbor_picked_[index] = 1;
            edge_points_indices_in_cloud_.push_back(index);
            // mark the neighbors as picked
            for (int l = 1; l <= num_of_neighbors_to_cluster_/2; ++l) {
                if (index + l >= cloud->points.size()) {
                    break;
                }
                float distance = sqrt(pow(cloud->points[index + l].x - cloud->points[index + l - 1].x, 2) +
                                      pow(cloud->points[index + l].y - cloud->points[index + l - 1].y, 2) +
                                      pow(cloud->points[index + l].z - cloud->points[index + l - 1].z, 2));
                if (distance > clustering_distance_threshold_) {
                    break;
                }
                cloud_neighbor_picked_[index + l] = 1;
            }

            for (int l = 1; l <= num_of_neighbors_to_cluster_/2; ++l) {
                if (index - l < 0) {
                    break;
                }
                float distance = sqrt(pow(cloud->points[index - l].x - cloud->points[index - l + 1].x, 2) +
                                      pow(cloud->points[index - l].y - cloud->points[index - l + 1].y, 2) +
                                      pow(cloud->points[index - l].z - cloud->points[index - l + 1].z, 2));
                if (distance > clustering_distance_threshold_) {
                    break;
                }
                cloud_neighbor_picked_[index - l] = 1;
            }
        }
    }
}

void InterestingDirectionExtractor::getInfoOfEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    Eigen::Vector3d last_point, next_point;
    for (int i = 0; i < edge_points_.size(); ++i) {
        EdgePoint edge_point;
        edge_point.position = edge_points_[i];
        // the point in cloud are sorted based on the angle in the xy plane
        // compare the distacne between the edge point and its two neighbors to determine if there is an obstacle on the left side
        if(edge_points_indices_in_cloud_[i] + 1 < cloud->points.size()){
            next_point = Eigen::Vector3d(cloud->points[edge_points_indices_in_cloud_[i] + 1].x,
                                         cloud->points[edge_points_indices_in_cloud_[i] + 1].y,
                                         cloud->points[edge_points_indices_in_cloud_[i] + 1].z);
        } else {
            next_point = Eigen::Vector3d(cloud->points[0].x,
                                         cloud->points[0].y,
                                         cloud->points[0].z);
        }
        
        if(edge_points_indices_in_cloud_[i] - 1 >= 0){
            last_point = Eigen::Vector3d(cloud->points[edge_points_indices_in_cloud_[i] - 1].x,
                                         cloud->points[edge_points_indices_in_cloud_[i] - 1].y,
                                         cloud->points[edge_points_indices_in_cloud_[i] - 1].z);
        } else {
            last_point = Eigen::Vector3d(cloud->points[cloud->points.size() - 1].x,
                                         cloud->points[cloud->points.size() - 1].y,
                                         cloud->points[cloud->points.size() - 1].z);
        }
        Eigen::Vector3d vec1 = edge_point.position - last_point;
        Eigen::Vector3d vec2 = next_point - edge_point.position;
        if (vec1.norm() - vec2.norm() > clustering_distance_threshold_) {
            edge_point.left_obstacle = false;
            edge_point.right_obstacle = true;
        } else if (vec2.norm() - vec1.norm() > clustering_distance_threshold_) {
            edge_point.left_obstacle = true;
            edge_point.right_obstacle = false;
        } else {
            edge_point.left_obstacle = true;
            edge_point.right_obstacle = true;
        }
        edge_points_with_info_.push_back(edge_point);
    }
}