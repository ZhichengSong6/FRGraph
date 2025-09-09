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

    edge_pts_pub_ = node_.advertise<visualization_msgs::Marker>("/edge_points", 10);
    direction_2d_pub_ = node_.advertise<visualization_msgs::Marker>("/interesting_directions_2d", 10);

    visualization_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::visualizationCallback, this);
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
    getInfoOfEdgePoints(cloud);
    sortEdgePoints();
    ExtractDirectionsFromEdgePoints();
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
    // base on the size of the cloud, resize the vectors
    curvatures_.resize(cloud->points.size(), 0.0);
    cloud_neighbor_picked_.resize(cloud->points.size(), 0);
    curvature_sorted_indices_.resize(cloud->points.size(), 0);
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
        edge_point.index_in_cloud = edge_points_indices_in_cloud_[i];
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

void InterestingDirectionExtractor::sortEdgePoints(){
    // sort edge_points_with_info_ based on index_in_cloud
    // so that the edge points are in the same order as in the original cloud
    // edge points which have obstacles on different sides are considered as noise and removed
    for (int i = 0; i < edge_points_with_info_.size(); ++i) {
        if (edge_points_with_info_[i].left_obstacle && edge_points_with_info_[i].right_obstacle) {
            edge_points_with_info_.erase(edge_points_with_info_.begin() + i);
            --i;
        }
    }
    std::sort(edge_points_with_info_.begin(), edge_points_with_info_.end(), [](const EdgePoint& a, const EdgePoint& b) {
        return a.index_in_cloud < b.index_in_cloud;
    });
}

void InterestingDirectionExtractor::ExtractDirectionsFromEdgePoints(){
    if(env_type_){
        /*code*/
    }
    else {
        // based on the edge points, extract the interesting directions
        // if two adjacent edge points have obstacles on different sides, then the direction between them is interesting
        interesting_pts_2d_.clear();
        for (int i = 0; i < edge_points_with_info_.size(); ++i) {
            int next_index = (i + 1) % edge_points_with_info_.size();
// std::cout << "i and next: " << i << " " << next_index << std::endl;
// std::cout << "right and left: " << edge_points_with_info_[i].right_obstacle << " " << edge_points_with_info_[next_index].left_obstacle << std::endl;
// std::cout << std::endl;
            // the edge points are sorted based on the index in the original cloud, which means they are sorted counter-clockwise
            // only need to check the right_obstacle of the current point and the left_obstacle of the next point
            if (!edge_points_with_info_[i].right_obstacle && !edge_points_with_info_[next_index].left_obstacle) {
// std::cout << "Found interesting direction between edge points " << i << " and " << next_index << std::endl;
                // length of the direction is based on the average distance between the two edge points and the origin
                double length = (edge_points_with_info_[i].position.head<2>().norm() + edge_points_with_info_[next_index].position.head<2>().norm()) / 2.0;
                // also need to check the angle between the two edge points
                double angle1 = atan2(edge_points_with_info_[i].position[1], edge_points_with_info_[i].position[0]);
                double angle2 = atan2(edge_points_with_info_[next_index].position[1], edge_points_with_info_[next_index].position[0]);
                
                // currently the angle are in [-pi, pi], need to convert them to [0, 2pi]
                if (angle1 < 0) angle1 += 2 * M_PI;
                if (angle2 < 0) angle2 += 2 * M_PI; 

                double angle_diff = angle2 - angle1;
                double angle_diff_lower_threshold = 0.05;
                double angle_diff_upper_threshold = 3.14;
                double angle_bias = 0.15; // to avoid the direction being too close to the edge points
                // if the error is too small, then the direction needs a small bias to generate polyhedron in the following step
// std::cout << "angle1, angle2, angle_diff: " << angle1 << " " << angle2 << " " << angle_diff << std::endl;
                if (angle_diff < angle_diff_lower_threshold){
                    // the direction is bias towards the direction with longer distance to the origin
                    if (edge_points_with_info_[i].position.head<2>().norm() > edge_points_with_info_[next_index].position.head<2>().norm()){
                        double angle = angle1 - angle_bias;
                        interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle), length * sin(angle)));
                    } else {
                        double angle = angle2 + angle_bias;
                        interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle), length * sin(angle)));
                    } 
                }
                else if(angle_diff > angle_diff_upper_threshold){
                    // the error is too large, which means the two edge points are on the opposite sides of the origin
                    // we consider three directions: the two directions from the origin to the two edge points with additional bias, and the direction opposite to the average direction of the two edge points
                    double average_angle = (angle1 + angle2) / 2.0;
                    interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle1 + angle_bias), length * sin(angle1 + angle_bias)));
                    interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle2 - angle_bias), length * sin(angle2 - angle_bias)));
                    interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(average_angle + M_PI), length * sin(average_angle + M_PI)));
                }
                else{
                    double average_angle = (angle1 + angle2) / 2.0;
                    interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(average_angle), length * sin(average_angle)));
                }
            }
        }
    }
}

void InterestingDirectionExtractor::publishEdgePoints() {
    visualization_msgs::Marker edge_points_marker;
    edge_points_marker.header.frame_id = "scan";
    edge_points_marker.header.stamp = ros::Time::now();
    edge_points_marker.ns = "edge_points";
    edge_points_marker.id = 0;
    edge_points_marker.type = visualization_msgs::Marker::POINTS;
    edge_points_marker.action = visualization_msgs::Marker::ADD;
    edge_points_marker.scale.x = 0.1;
    edge_points_marker.scale.y = 0.1;
    edge_points_marker.color.r = 1.0;
    edge_points_marker.color.g = 0.0;
    edge_points_marker.color.b = 0.0;
    edge_points_marker.color.a = 1.0;

    geometry_msgs::Point p1, p2;
    p1.x = 0.0;
    p1.y = 0.0;
    p1.z = 0.0;
    for (const auto &edge_point : edge_points_with_info_) {
        p2.x = edge_point.position[0];
        p2.y = edge_point.position[1];
        p2.z = edge_point.position[2];
        edge_points_marker.points.push_back(p2);
        // edge_points_marker.points.push_back(p1); // line from edge point to origin
    }

    edge_pts_pub_.publish(edge_points_marker);
}

void InterestingDirectionExtractor::publishInterestingDirections2D() {
    // publish the interesting directions in 2D as visualization markers
    visualization_msgs::Marker direction_2d_marker;
    direction_2d_marker.header.frame_id = "scan";
    direction_2d_marker.header.stamp = ros::Time::now();
    direction_2d_marker.ns = "interesting_directions_2d";
    direction_2d_marker.id = 0;
    direction_2d_marker.type = visualization_msgs::Marker::LINE_LIST;
    direction_2d_marker.action = visualization_msgs::Marker::ADD;
    direction_2d_marker.scale.x = 0.01; // shaft diameter

    direction_2d_marker.color.r = 0.0;
    direction_2d_marker.color.g = 1.0;
    direction_2d_marker.color.b = 0.0;
    direction_2d_marker.color.a = 1.0;

    geometry_msgs::Point p_start, p_end;
    p_start.x = 0.0;
    p_start.y = 0.0;
    p_start.z = 0.0;
    for (const auto &dir : interesting_pts_2d_) {
        p_end.x = dir[0];
        p_end.y = dir[1];
        p_end.z = 0.0;
        direction_2d_marker.points.push_back(p_start);
        direction_2d_marker.points.push_back(p_end);
    }
    direction_2d_pub_.publish(direction_2d_marker);
}

void InterestingDirectionExtractor::visualizationCallback(const ros::TimerEvent &e) {
    publishEdgePoints();
    if (!env_type_) {
        publishInterestingDirections2D();
    }
}