#include "interesting_directions_extractor/interesting_directions_extractor.h"

void InterestingDirectionExtractor::initialize(ros::NodeHandle &nh, bool env_type)
{
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

    ROS_INFO("InterestingDirectionExtractor initialized, env_type: %d", env_type_);

    // set empty point cloud
    pointcloud_cropped_scan_frame_.clear();
    pointcloud_cropped_scan_frame_2d_.clear();

    if (env_type_)
    {
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &InterestingDirectionExtractor::velodyneCallback, this);
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections3D, this);
        // goal_sub_ = node_.subscribe("/goal", 1, &InterestingDirectionExtractor::goalCallback, this);
        goal_sub_ = node_.subscribe("/navigation_goal_3d", 1, &InterestingDirectionExtractor::goalCallback, this);
    }
    else
    {
        scan2d_sub_ = node_.subscribe("/scan", 1, &InterestingDirectionExtractor::scan2dCallback, this);
        direction_extraction_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::extractInterestingDirections2D, this);
        goal_sub_ = node_.subscribe("/goal", 1, &InterestingDirectionExtractor::goalCallback, this);
    }

    lidar_to_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf2_ros::Buffer tfBuffer_lidar;
    tf2_ros::TransformListener tfListener_lidar(tfBuffer_lidar);
    try
    {
        *(lidar_to_base_ptr_) = tfBuffer_lidar.lookupTransform("base_link", "scan", ros::Time::now(), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    const Eigen::Affine3d T_lidar_base = tf2::transformToEigen(*lidar_to_base_ptr_);
    T_lidar_base_mat_ = T_lidar_base.matrix().cast<float>();

    base_to_odom_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf_listener_odom_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_odom_);

    odom_timer_ = node_.createTimer(ros::Duration(0.1), &InterestingDirectionExtractor::odomTimerCallback, this);

    edge_pts_pub_ = node_.advertise<visualization_msgs::Marker>("/edge_points", 10);
    direction_2d_pub_ = node_.advertise<visualization_msgs::Marker>("/interesting_directions_2d", 10);

    temp_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/temp_map", 10);
    point_pub_ = node_.advertise<visualization_msgs::Marker>("/point_for_debug", 10);

    visualization_timer_ = node_.createTimer(ros::Duration(0.05), &InterestingDirectionExtractor::visualizationCallback, this);
}

void InterestingDirectionExtractor::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
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
    for (const auto &pt : pointcloud)
    {
        if (pt[0] > -size_of_cropped_pointcloud_[0] / 2 && pt[0] < size_of_cropped_pointcloud_[0] / 2 &&
            pt[1] > -size_of_cropped_pointcloud_[1] / 2 && pt[1] < size_of_cropped_pointcloud_[1] / 2 &&
            pt[2] > -size_of_cropped_pointcloud_[2] / 2 && pt[2] < size_of_cropped_pointcloud_[2] / 2)
        {
            pointcloud_cropped.push_back(pt);
        }
    }

    // pointcloud are in scan frame
    pointcloud_cropped_scan_frame_ = pointcloud_cropped;
}

void InterestingDirectionExtractor::scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    // Process the 2D laser scan data
    if (msg->ranges.size() == 0)
    {
        ROS_WARN("[InterestingDirectionExtractor] Received empty laser scan data");
        return;
    }
    vec_Vec2f pointcloud_2d;
    double angle = msg->angle_min;
    for (const auto &range : msg->ranges)
    {
        if (range < msg->range_max && range > msg->range_min)
        {
            double x = range * cos(angle);
            double y = range * sin(angle);
            pointcloud_2d.push_back(Eigen::Vector2d(x, y));
        }
        angle += msg->angle_increment;
    }

    vec_Vec2f pointcloud_cropped_2d;

    // crop the point cloud
    for (const auto &pt : pointcloud_2d)
    {
        if (pt[0] > -size_of_cropped_pointcloud_[0] / 2 && pt[0] < size_of_cropped_pointcloud_[0] / 2 &&
            pt[1] > -size_of_cropped_pointcloud_[1] / 2 && pt[1] < size_of_cropped_pointcloud_[1] / 2)
        {
            pointcloud_cropped_2d.push_back(pt);
        }
    }
    // pointcloud are in scan frame
    pointcloud_cropped_scan_frame_2d_ = pointcloud_cropped_2d;
}

void InterestingDirectionExtractor::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_ori_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    quaternionToRPY(goal_ori_, goal_roll_, goal_pitch_, goal_yaw_);
}

void InterestingDirectionExtractor::odomTimerCallback(const ros::TimerEvent &event)
{
    try
    {
        *base_to_odom_ptr_ = tfBuffer_odom_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void InterestingDirectionExtractor::extractInterestingDirections2D(const ros::TimerEvent &e)
{
    // Implementation of the direction extraction logic
    if (pointcloud_cropped_scan_frame_2d_.empty())
    {
        // if the point cloud is empty, no obstacle detected
        // add four directions +x, -x, +y, -y, length is defined by the cropped pointcloud size
        // addtionally, add the goal direction if the goal is set
        interesting_pts_2d_.clear();
        edge_points_with_info_.clear();
        double length = std::min(size_of_cropped_pointcloud_[0] / 2, size_of_cropped_pointcloud_[1] / 2);
        interesting_pts_2d_.push_back(Eigen::Vector2d(length, 0.0));
        interesting_pts_2d_.push_back(Eigen::Vector2d(-length, 0.0));
        interesting_pts_2d_.push_back(Eigen::Vector2d(0.0, length));
        interesting_pts_2d_.push_back(Eigen::Vector2d(0.0, -length));

        // the goal is in odom frame, need to convert it to scan frame
        const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
        Eigen::Matrix4d T_base_odom_mat = T_base_odom.matrix().cast<double>();
        Eigen::Matrix4d T_odom_base_mat = T_base_odom_mat.inverse();
        Eigen::Matrix4d T_lidar_base_mat = T_lidar_base_mat_.cast<double>();
        Eigen::Matrix4d T_base_lidar_mat = T_lidar_base_mat.inverse();
        Eigen::Matrix4d T_scan_odom_mat = T_base_lidar_mat * T_odom_base_mat;
        Eigen::Vector4d goal_pos_homogeneous(goal_pos_[0], goal_pos_[1], goal_pos_[2], 1.0);
        Eigen::Vector4d goal_pos_in_scan_homogeneous = T_scan_odom_mat * goal_pos_homogeneous;
        Eigen::Vector3d goal_in_scan_frame = goal_pos_in_scan_homogeneous.head<3>() / goal_pos_in_scan_homogeneous[3];
        goal_in_scan_frame[2] = 0.0; // project to 2D plane
        if (goal_in_scan_frame.norm() < length)
        {
            interesting_pts_2d_.push_back(Eigen::Vector2d(goal_in_scan_frame[0], goal_in_scan_frame[1]));
        }
        else
        {
            interesting_pts_2d_.push_back(Eigen::Vector2d(goal_in_scan_frame.normalized()[0] * length, goal_in_scan_frame.normalized()[1] * length));
        }
        // check if any direction is too close to each other, if so, remove one
        // notice that the direction towards the goal is always kept
        std::vector<Eigen::Vector2d> interesting_pts_2d_filtered;
        interesting_pts_2d_filtered.push_back(interesting_pts_2d_.back()); // always keep the goal direction
        for (int i = 0; i < interesting_pts_2d_.size() - 1; ++i)
        {
            bool too_close = false;
            for (const auto &pt : interesting_pts_2d_filtered)
            {
                // use angle to check if two directions are too close, threshold is 30 degrees
                double angle = acos(interesting_pts_2d_[i].dot(pt) / (interesting_pts_2d_[i].norm() * pt.norm()));
                if (angle < M_PI / 6)
                {
                    too_close = true;
                    break;
                }
            }
            if (!too_close)
            {
                interesting_pts_2d_filtered.push_back(interesting_pts_2d_[i]);
            }
        }
        interesting_pts_2d_ = interesting_pts_2d_filtered;
    }
    else
    {
        interesting_pts_2d_.clear();
        edge_points_with_info_.clear();
        // convert vec_Vec2f to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto &pt : pointcloud_cropped_scan_frame_2d_)
        {
            cloud->points.push_back(pcl::PointXYZ(pt[0], pt[1], 0.0));
        }
    
        extractEdgePoints(cloud, num_of_edge_pts_);
        getInfoOfEdgePoints(cloud);
        sortEdgePoints();
ROS_INFO("Number of edge points extracted: %lu", edge_points_with_info_.size());
        checkGoalPose2D();
ROS_INFO("Number of edge points after adding goal (if any): %lu", edge_points_with_info_.size());
        ExtractDirectionsFromEdgePoints();
    }

}

void InterestingDirectionExtractor::extractInterestingDirections3D(const ros::TimerEvent &e)
{
    // Implementation of the direction extraction logic
    if (pointcloud_cropped_scan_frame_.empty())
    {
        ROS_INFO("Empty cropped 3D point cloud, skip direction extraction");
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(new pcl::PointCloud<pcl::PointXYZ>());
    processPointCloudTo2D(cloud_in_base);
    
    if (cloud_in_base->empty())
    {
        potential_directions_3d_.direction_to_free_space.clear();
        potential_directions_3d_.direction_to_obstacle.clear();
        // if the point cloud is empty, no obstacle detected
        // add four directions +x, -x, +y, -y, length is defined by the cropped pointcloud size
        // addtionally, add the goal direction if the goal is set
        double length = std::min(size_of_cropped_pointcloud_[0] / 2, size_of_cropped_pointcloud_[1] / 2);
        potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(length, 0.0, 0.0));
        potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(-length, 0.0, 0.0));
        potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(0.0, length, 0.0));
        potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(0.0, -length, 0.0));
        
        // the goal is in odom frame, need to convert it to base frame
        const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
        Eigen::Matrix4d T_base_odom_mat = T_base_odom.matrix().cast<double>();
        Eigen::Matrix4d T_odom_base_mat = T_base_odom_mat.inverse();
        Eigen::Vector4d goal_pos_homogeneous(goal_pos_[0], goal_pos_[1], goal_pos_[2], 1.0);
        Eigen::Vector4d goal_pos_in_base_homogeneous = T_odom_base_mat * goal_pos_homogeneous;
        Eigen::Vector3d goal_in_base_frame = goal_pos_in_base_homogeneous.head<3>() / goal_pos_in_base_homogeneous[3];
        goal_in_base_frame[2] = 0.0; // project the goal to the robot's height plane
        if (goal_in_base_frame.norm() < length)
        {
            potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(goal_in_base_frame[0], goal_in_base_frame[1], goal_in_base_frame[2]));
        }
        else
        {
            potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(goal_in_base_frame.normalized()[0] * length, goal_in_base_frame.normalized()[1] * length, 0.0));
        }
        // check if any direction is too close to each other, if so, remove one
        // notice that the direction towards the goal is always kept
        std::vector<Eigen::Vector3d> interesting_pts_3d_filtered;
        interesting_pts_3d_filtered.push_back(potential_directions_3d_.direction_to_free_space.back()); // always keep the goal direction
        for (int i = 0; i < potential_directions_3d_.direction_to_free_space.size() - 1; ++i)
        {
            bool too_close = false;
            for (const auto &pt : interesting_pts_3d_filtered)
            {
                // use angle to check if two directions are too close, threshold is 30 degrees
                double angle = acos(potential_directions_3d_.direction_to_free_space[i].dot(pt) / (potential_directions_3d_.direction_to_free_space[i].norm() * pt.norm()));
                if (angle < M_PI / 6)
                {
                    too_close = true;
                    break;
                }
            }
            if (!too_close)
            {
                interesting_pts_3d_filtered.push_back(potential_directions_3d_.direction_to_free_space[i]);
            }
        }
ROS_INFO("Number of interesting directions in 3D after filtering: %lu", interesting_pts_3d_filtered.size());
        potential_directions_3d_.direction_to_free_space = interesting_pts_3d_filtered;
    }
    else{
        interesting_pts_3d_.clear();
        edge_points_with_info_.clear();
        potential_directions_3d_.direction_to_free_space.clear();
        potential_directions_3d_.direction_to_obstacle.clear();

        // base on the projected 2D point cloud, extract edge points
        extractEdgePoints(cloud_in_base, num_of_edge_pts_);
        getInfoOfEdgePoints(cloud_in_base);
        sortEdgePoints();
// ROS_INFO("Number of edge points extracted: %lu", edge_points_with_info_.size());
        checkGoalPose3D1();
        ExtractDirectionsFromEdgePoints();
    }

}

void InterestingDirectionExtractor::extractEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int num_edges)
{
    if (cloud->points.size() == 0)
    {
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

void InterestingDirectionExtractor::computeCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (cloud->points.size() < 5)
    {
        ROS_WARN("[InterestingDirectionExtractor] Not enough points to compute curvature");
        return;
    }

    for (int i = 5; i < cloud->points.size() - 5; ++i)
    {
        float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 10 * cloud->points[i].x + cloud->points[i + 5].x + cloud->points[i + 4].x + cloud->points[i + 3].x + cloud->points[i + 2].x + cloud->points[i + 1].x;
        float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 10 * cloud->points[i].y + cloud->points[i + 5].y + cloud->points[i + 4].y + cloud->points[i + 3].y + cloud->points[i + 2].y + cloud->points[i + 1].y;
        float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 10 * cloud->points[i].z + cloud->points[i + 5].z + cloud->points[i + 4].z + cloud->points[i + 3].z + cloud->points[i + 2].z + cloud->points[i + 1].z;
        curvatures_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }

    // get the curvature of the first 5 points
    for (int i = 0; i < 5; ++i)
    {
        float diffX = cloud->points[i + 1].x + cloud->points[i + 2].x + cloud->points[i + 3].x + cloud->points[i + 4].x + cloud->points[i + 5].x - 10 * cloud->points[i].x;
        float diffY = cloud->points[i + 1].y + cloud->points[i + 2].y + cloud->points[i + 3].y + cloud->points[i + 4].y + cloud->points[i + 5].y - 10 * cloud->points[i].y;
        float diffZ = cloud->points[i + 1].z + cloud->points[i + 2].z + cloud->points[i + 3].z + cloud->points[i + 4].z + cloud->points[i + 5].z - 10 * cloud->points[i].z;

        // base on the index of the point, add certain points from the end of the cloud to make sure 10 points are used
        for (int j = 0; j < 5 - i; ++j)
        {
            diffX += cloud->points[cloud->points.size() - 1 - j].x;
            diffY += cloud->points[cloud->points.size() - 1 - j].y;
            diffZ += cloud->points[cloud->points.size() - 1 - j].z;
        }
        for (int j = 0; j < i; ++j)
        {
            diffX += cloud->points[j].x;
            diffY += cloud->points[j].y;
            diffZ += cloud->points[j].z;
        }
        curvatures_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }
    // get the curvature of the last 5 points
    for (int i = cloud->points.size() - 5; i < cloud->points.size(); ++i)
    {
        float diffX = cloud->points[i - 5].x + cloud->points[i - 4].x + cloud->points[i - 3].x + cloud->points[i - 2].x + cloud->points[i - 1].x - 10 * cloud->points[i].x;
        float diffY = cloud->points[i - 5].y + cloud->points[i - 4].y + cloud->points[i - 3].y + cloud->points[i - 2].y + cloud->points[i - 1].y - 10 * cloud->points[i].y;
        float diffZ = cloud->points[i - 5].z + cloud->points[i - 4].z + cloud->points[i - 3].z + cloud->points[i - 2].z + cloud->points[i - 1].z - 10 * cloud->points[i].z;

        // base on the index of the point, add certain points from the beginning of the cloud to make sure 10 points are used
        for (int j = 0; j < i - (cloud->points.size() - 5) + 1; ++j)
        {
            diffX += cloud->points[j].x;
            diffY += cloud->points[j].y;
            diffZ += cloud->points[j].z;
        }
        for (int j = 0; j < 5 - (i - (cloud->points.size() - 5) + 1); ++j)
        {
            diffX += cloud->points[cloud->points.size() - 1 - j].x;
            diffY += cloud->points[cloud->points.size() - 1 - j].y;
            diffZ += cloud->points[cloud->points.size() - 1 - j].z;
        }
        curvatures_[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_neighbor_picked_[i] = 0;
        curvature_sorted_indices_[i] = i;
    }
}

void InterestingDirectionExtractor::sortIndicesBasedOnCurvature()
{
    // sort cloud_sort_ind_ based on curvatures
    std::sort(curvature_sorted_indices_.begin(), curvature_sorted_indices_.end(), [&](const int &a, const int &b)
              { return curvatures_[a] > curvatures_[b]; });
}

void InterestingDirectionExtractor::selectEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int num_points)
{
    int count = 0;
    for (int k = 0; k < curvature_sorted_indices_.size(); ++k)
    {
        int index = curvature_sorted_indices_[k];
        if (cloud_neighbor_picked_[index] == 0 && curvatures_[index] > curvature_threshold_)
        {
            count++;
            if (count < num_points)
            {
                edge_points_.push_back(Eigen::Vector3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z));
            }
            else
            {
                break;
            }
            cloud_neighbor_picked_[index] = 1;
            edge_points_indices_in_cloud_.push_back(index);
            // mark the neighbors as picked
            for (int l = 1; l <= num_of_neighbors_to_cluster_ / 2; ++l)
            {
                if (index + l >= cloud->points.size())
                {
                    break;
                }
                float distance = sqrt(pow(cloud->points[index + l].x - cloud->points[index + l - 1].x, 2) +
                                      pow(cloud->points[index + l].y - cloud->points[index + l - 1].y, 2) +
                                      pow(cloud->points[index + l].z - cloud->points[index + l - 1].z, 2));
                if (distance > clustering_distance_threshold_)
                {
                    break;
                }
                cloud_neighbor_picked_[index + l] = 1;
            }

            for (int l = 1; l <= num_of_neighbors_to_cluster_ / 2; ++l)
            {
                if (index - l < 0)
                {
                    break;
                }
                float distance = sqrt(pow(cloud->points[index - l].x - cloud->points[index - l + 1].x, 2) +
                                      pow(cloud->points[index - l].y - cloud->points[index - l + 1].y, 2) +
                                      pow(cloud->points[index - l].z - cloud->points[index - l + 1].z, 2));
                if (distance > clustering_distance_threshold_)
                {
                    break;
                }
                cloud_neighbor_picked_[index - l] = 1;
            }
        }
    }
}

void InterestingDirectionExtractor::getInfoOfEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::Vector3d last_point, next_point;
    for (int i = 0; i < edge_points_.size(); ++i)
    {
        EdgePoint edge_point;
        edge_point.position = edge_points_[i];
        edge_point.index_in_cloud = edge_points_indices_in_cloud_[i];
        // the point in cloud are sorted based on the angle in the xy plane
        // compare the distacne between the edge point and its two neighbors to determine if there is an obstacle on the left side
        if (edge_points_indices_in_cloud_[i] + 1 < cloud->points.size())
        {
            next_point = Eigen::Vector3d(cloud->points[edge_points_indices_in_cloud_[i] + 1].x,
                                         cloud->points[edge_points_indices_in_cloud_[i] + 1].y,
                                         cloud->points[edge_points_indices_in_cloud_[i] + 1].z);
        }
        else
        {
            next_point = Eigen::Vector3d(cloud->points[0].x,
                                         cloud->points[0].y,
                                         cloud->points[0].z);
        }

        if (edge_points_indices_in_cloud_[i] - 1 >= 0)
        {
            last_point = Eigen::Vector3d(cloud->points[edge_points_indices_in_cloud_[i] - 1].x,
                                         cloud->points[edge_points_indices_in_cloud_[i] - 1].y,
                                         cloud->points[edge_points_indices_in_cloud_[i] - 1].z);
        }
        else
        {
            last_point = Eigen::Vector3d(cloud->points[cloud->points.size() - 1].x,
                                         cloud->points[cloud->points.size() - 1].y,
                                         cloud->points[cloud->points.size() - 1].z);
        }
        Eigen::Vector3d vec1 = edge_point.position - last_point;
        Eigen::Vector3d vec2 = next_point - edge_point.position;
        if (vec1.norm() > clustering_distance_threshold_ && vec2.norm() > clustering_distance_threshold_){
            // both sides are too far, consider it as noise, skip this point
            continue;
        }
        else if (vec1.norm() - vec2.norm() > clustering_distance_threshold_)
        {
            edge_point.left_obstacle = false;
            edge_point.right_obstacle = true;
        }
        else if (vec2.norm() - vec1.norm() > clustering_distance_threshold_)
        {
            edge_point.left_obstacle = true;
            edge_point.right_obstacle = false;
        }
        else
        {
            edge_point.left_obstacle = true;
            edge_point.right_obstacle = true;
        }
        edge_points_with_info_.push_back(edge_point);
    }
}

void InterestingDirectionExtractor::sortEdgePoints()
{
    // edge points which have obstacles on different sides are considered as noise and removed
    for (int i = 0; i < edge_points_with_info_.size(); ++i)
    {
        if (edge_points_with_info_[i].left_obstacle && edge_points_with_info_[i].right_obstacle)
        {
            edge_points_with_info_.erase(edge_points_with_info_.begin() + i);
            --i;
        }
    }
    // for convience, let the points sort in counter-clockwise and start from the positive x axis
    std::vector<double> angles;
    for (const auto &edge_point : edge_points_with_info_){
        double angle = atan2(edge_point.position[1], edge_point.position[0]);
        if (angle < 0){
            angle += 2 * M_PI;
        }
        angles.push_back(angle);
    }
    // sort edge_points_with_info_ based on angles
    std::vector<int> indices(edge_points_with_info_.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int a, int b) { return angles[a] < angles[b]; });
    std::vector<EdgePoint> sorted_edge_points;
    for (const auto &index : indices){
        sorted_edge_points.push_back(edge_points_with_info_[index]);
    }
    edge_points_with_info_ = sorted_edge_points;
}

void InterestingDirectionExtractor::checkGoalPose2D()
{
    if (base_to_odom_ptr_ == nullptr)
    {
        ROS_WARN("[InterestingDirectionExtractor] base_to_odom_ptr_ is null");
        return;
    }
    if (edge_points_with_info_.size() == 0){
        ROS_WARN("[InterestingDirectionExtractor] No Edge Point");
    }
    if (goal_pos_ == Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        // ROS_WARN("[InterestingDirectionExtractor] Goal is not set");
        return;
    }
    // the goal is in odom frame, need to convert it to scan frame
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    Eigen::Matrix4d T_base_odom_mat = T_base_odom.matrix().cast<double>();
    Eigen::Matrix4d T_odom_base_mat = T_base_odom_mat.inverse();
    Eigen::Matrix4d T_lidar_base_mat = T_lidar_base_mat_.cast<double>();
    Eigen::Matrix4d T_base_lidar_mat = T_lidar_base_mat.inverse();
    Eigen::Matrix4d T_scan_odom_mat = T_base_lidar_mat * T_odom_base_mat;
    Eigen::Vector4d goal_pos_homogeneous(goal_pos_[0], goal_pos_[1], goal_pos_[2], 1.0);
    Eigen::Vector4d goal_pos_in_scan_homogeneous = T_scan_odom_mat * goal_pos_homogeneous;
    Eigen::Vector3d goal_in_scan_frame = goal_pos_in_scan_homogeneous.head<3>() / goal_pos_in_scan_homogeneous[3];

    // compute the angle of the goal_in_scan_frame to find the edge points that next to it
    double goal_angle = atan2(goal_in_scan_frame[1], goal_in_scan_frame[0]);
    if (goal_angle < 0)
    {
        goal_angle += 2 * M_PI;
    }
    int index = edge_points_with_info_.size(); // index of the first edge point that has angle larger than goal_angle
    for (int i = 0; i < edge_points_with_info_.size(); ++i)
    {
        double edge_point_angle = atan2(edge_points_with_info_[i].position[1], edge_points_with_info_[i].position[0]);
        if (edge_point_angle < 0)
        {
            edge_point_angle += 2 * M_PI;
        }
        if (edge_point_angle >= goal_angle)
        {
            index = i;
            break;
        }
    }
    int prev_index = (index - 1 + edge_points_with_info_.size()) % edge_points_with_info_.size();
    // if the two edge points have obstacles on different sides, and the distance to the origin less than the cropped pointcloud
    if (!edge_points_with_info_[index].left_obstacle && !edge_points_with_info_[prev_index].right_obstacle)
    {
        if(goal_in_scan_frame.norm() < std::min(size_of_cropped_pointcloud_[0]/2, size_of_cropped_pointcloud_[1]/2)){
            EdgePoint goal_edge_point;
            goal_edge_point.position = goal_in_scan_frame;
            goal_edge_point.index_in_cloud = -1; // goal point does not have index in cloud
            goal_edge_point.left_obstacle = false;
            goal_edge_point.right_obstacle = false;
            edge_points_with_info_.insert(edge_points_with_info_.begin() + index, goal_edge_point);
            goal_added_ = true;
        }
        else{
            // the goal is out of the cropped pointcloud, add a point in the direction of the goal with the distance of the cropped pointcloud
            EdgePoint goal_edge_point;
            goal_edge_point.position = goal_in_scan_frame.normalized() * std::min(size_of_cropped_pointcloud_[0]/2, size_of_cropped_pointcloud_[1]/2);
            goal_edge_point.index_in_cloud = -1; // goal point does not have index in cloud
            goal_edge_point.left_obstacle = false;
            goal_edge_point.right_obstacle = false;
            edge_points_with_info_.insert(edge_points_with_info_.begin() + index, goal_edge_point);
            goal_added_ = true;
        }
    }
    else{
        // there is obstacle between two edge points
        // if the goal point's distance to the origin is the smallest, add the goal points
        if (goal_in_scan_frame.norm() < std::min(edge_points_with_info_[index].position.norm(), edge_points_with_info_[prev_index].position.norm())){
            EdgePoint goal_edge_point;
            goal_edge_point.position = goal_in_scan_frame;
            goal_edge_point.index_in_cloud = -1; // goal point does not have index in cloud
            goal_edge_point.left_obstacle = false;
            goal_edge_point.right_obstacle = false;
            edge_points_with_info_.insert(edge_points_with_info_.begin() + index, goal_edge_point);
            goal_added_ = true;
        }
    }
}

void InterestingDirectionExtractor::processPointCloudTo2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base)
{
    // convert vec_Vec3f to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pt : pointcloud_cropped_scan_frame_)
    {
        cloud->points.push_back(pcl::PointXYZ(pt[0], pt[1], pt[2]));
    }
    // the cloud are in scan frame, need to transform to base frame

    pcl::transformPointCloud(*cloud, *cloud_in_base, T_lidar_base_mat_);
    // extract the pointcloud data that are around the height of the robot
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in_base);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.06, 0.06);
    pass.filter(*cloud_in_base);
    // put all points into same height
    for (auto &pt : cloud_in_base->points)
    {
        pt.z = 0;
    }
    // the projection change the order of the points, so we need to re-organize the point cloud based on the angle to the origin
    std::vector<std::pair<double, int>> angle_index_pairs;
    for (int i = 0; i < cloud_in_base->points.size(); ++i)
    {
        double angle = atan2(cloud_in_base->points[i].y, cloud_in_base->points[i].x);
        if (angle < 0){
            angle += 2 * M_PI;
        }
        angle_index_pairs.push_back(std::make_pair(angle, i));
    }
    std::sort(angle_index_pairs.begin(), angle_index_pairs.end());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base_sorted(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &pair : angle_index_pairs)
    {
        cloud_in_base_sorted->points.push_back(cloud_in_base->points[pair.second]);
    }
    cloud_in_base_sorted->width = cloud_in_base_sorted->points.size();
    cloud_in_base_sorted->height = 1;
    cloud_in_base_sorted->is_dense = true;
    cloud_in_base = cloud_in_base_sorted;
    // store the temp map for visualization
    temp_map_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud_in_base);    
}

void InterestingDirectionExtractor::checkGoalPose3D1(){
    if (base_to_odom_ptr_ == nullptr)
    {
        ROS_WARN("[InterestingDirectionExtractor] base_to_odom_ptr_ is null");
        return;
    }
    if (edge_points_with_info_.size() == 0){
        ROS_WARN("[InterestingDirectionExtractor] No Edge Point");
    }
    if (goal_pos_ == Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        ROS_WARN("[InterestingDirectionExtractor] Goal is not set");
        return;
    }
    // the goal is in odom frame, need to convert it to base frame
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    Eigen::Matrix4d T_base_odom_mat = T_base_odom.matrix().cast<double>();
    Eigen::Matrix4d T_odom_base_mat = T_base_odom_mat.inverse();
    Eigen::Vector4d goal_pos_homogeneous(goal_pos_[0], goal_pos_[1], goal_pos_[2], 1.0);
    Eigen::Vector4d goal_pos_in_base_homogeneous = T_odom_base_mat * goal_pos_homogeneous;
    Eigen::Vector3d goal_in_base_frame = goal_pos_in_base_homogeneous.head<3>() / goal_pos_in_base_homogeneous[3];
    goal_in_base_frame[2] = 0.0; // project the goal to the robot's height plane

    // compute the angle of the goal_in_base_frame to find the edge points that next to it
    double goal_angle = atan2(goal_in_base_frame[1], goal_in_base_frame[0]);
    if (goal_angle < 0)
    {
        goal_angle += 2 * M_PI;
    }
    int index = edge_points_with_info_.size(); // index of the first edge point that has angle larger than goal_angle
    for (int i = 0; i < edge_points_with_info_.size(); ++i)
    {
        double edge_point_angle = atan2(edge_points_with_info_[i].position[1], edge_points_with_info_[i].position[0]);
        if (edge_point_angle < 0)
        {
            edge_point_angle += 2 * M_PI;
        }
        if (edge_point_angle >= goal_angle)
        {
            index = i;
            break;
        }
    }
    int prev_index = (index - 1 + edge_points_with_info_.size()) % edge_points_with_info_.size();
    // if the two edge points have obstacles on different sides, and the distance to the origin less than the cropped pointcloud
    if (!edge_points_with_info_[index].left_obstacle && !edge_points_with_info_[prev_index].right_obstacle)
    {
        if(goal_in_base_frame.norm() < std::min(size_of_cropped_pointcloud_[0]/2, size_of_cropped_pointcloud_[1]/2)){
            EdgePoint goal_edge_point;
            goal_edge_point.position = goal_in_base_frame;
            goal_edge_point.index_in_cloud = -1; // goal point does not have index in cloud
            goal_edge_point.left_obstacle = false;
            goal_edge_point.right_obstacle = false;
            edge_points_with_info_.insert(edge_points_with_info_.begin() + index, goal_edge_point);
            goal_added_ = true;
        }
    }
    else{
        // there is obstacle between two edge points
        // if the goal point's distance to the origin is the smallest, add the goal points
        if (goal_in_base_frame.norm() < std::min(edge_points_with_info_[index].position.norm(), edge_points_with_info_[prev_index].position.norm())){
            EdgePoint goal_edge_point;
            goal_edge_point.position = goal_in_base_frame;
            goal_edge_point.index_in_cloud = -1; // goal point does not have index in cloud
            goal_edge_point.left_obstacle = false;
            goal_edge_point.right_obstacle = false;
            edge_points_with_info_.insert(edge_points_with_info_.begin() + index, goal_edge_point);
            goal_added_ = true;
        }
    }
}

void InterestingDirectionExtractor::ExtractDirectionsFromEdgePoints()
{
    if (env_type_)
    {
        interesting_pts_3d_.clear();
        // first based on the edge points extracted in 2D, extract the interesting directions and direction that lead towards to obstacles
        extractDirectionsToFreeSpace();
        for(auto &pt_2d: interesting_pts_2d_){
            potential_directions_3d_.direction_to_free_space.push_back(Eigen::Vector3d(pt_2d[0], pt_2d[1], 0.0));
        }
        interesting_pts_2d_.clear();
        // also need to extract the direction that lead towards to obstacles

    }
    else
    {
        if (edge_points_with_info_.size() == 0)
        {
            ROS_INFO("No edge points detected");
            /* code */
        }
        else{
            extractDirectionsToFreeSpace();
        }
    }
}

void InterestingDirectionExtractor::extractDirectionsToFreeSpace()
{
    // based on the edge points, extract the interesting directions
    // if two adjacent edge points have obstacles on different sides, then the direction between them is interesting
    interesting_pts_2d_.clear();
    bool have_goal = false;
    double max_distance = std::min(size_of_cropped_pointcloud_[0], size_of_cropped_pointcloud_[1]) / 2.0;
    for (int i = 0; i < edge_points_with_info_.size(); ++i)
    {
        int next_index = (i + 1) % edge_points_with_info_.size();
// std::cout << "i and next: " << i << " " << next_index << std::endl;
// std::cout << "right and left: " << edge_points_with_info_[i].right_obstacle << " " << edge_points_with_info_[next_index].left_obstacle << std::endl;
// std::cout << std::endl;
        // the edge points are sorted based on the index in the original cloud, which means they are sorted counter-clockwise
        // only need to check the right_obstacle of the current point and the left_obstacle of the next point
        // need to pay attention to goal_point, whose index_in_cloud is -1
        if (edge_points_with_info_[i].index_in_cloud == -1 || edge_points_with_info_[next_index].index_in_cloud == -1)
        {
            if(have_goal){
                continue;
            }
            else{
                // if one of the two edge points is the goal point, directly set the goal points as interesting direction
                if (edge_points_with_info_[i].index_in_cloud == -1){
                    interesting_pts_2d_.push_back(Eigen::Vector2d(edge_points_with_info_[i].position[0], edge_points_with_info_[i].position[1]));
                    have_goal = true;
                }
                else if (edge_points_with_info_[next_index].index_in_cloud == -1){
                    interesting_pts_2d_.push_back(Eigen::Vector2d(edge_points_with_info_[next_index].position[0], edge_points_with_info_[next_index].position[1]));
                    have_goal = true;
                }
            }
        }
        else{
            if (!edge_points_with_info_[i].right_obstacle && !edge_points_with_info_[next_index].left_obstacle)
            {
                // also need to check the angle between the two edge points
                double angle1 = atan2(edge_points_with_info_[i].position[1], edge_points_with_info_[i].position[0]);
                double angle2 = atan2(edge_points_with_info_[next_index].position[1], edge_points_with_info_[next_index].position[0]);

                // currently the angle are in [-pi, pi], need to convert them to [0, 2pi]
                if (angle1 < 0)
                    angle1 += 2 * M_PI;
                if (angle2 < 0)
                    angle2 += 2 * M_PI;

                double angle_diff = angle2 - angle1;
                if (angle_diff < 0){
                    angle_diff += 2 * M_PI;
                }
                double angle_diff_lower_threshold = 0.05;
                double angle_diff_upper_threshold = 3.14;
                double angle_bias = 0.05; // to avoid the direction being too close to the edge points
                // if the error is too small, then the direction needs a small bias to generate polyhedron in the following step
                if (angle_diff < angle_diff_lower_threshold)
                {
                    // the direction is bias towards the direction with longer distance to the origin
                    // the length is set to the distance of the closer edge point to the origin
                    if (edge_points_with_info_[i].position.head<2>().norm() > edge_points_with_info_[next_index].position.head<2>().norm())
                    {
                        double angle = angle1 - angle_bias;
                        double length = edge_points_with_info_[next_index].position.head<2>().norm();
                        interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle), length * sin(angle)));
                    }
                    else
                    {
                        double angle = angle2 + angle_bias;
                        double length = edge_points_with_info_[i].position.head<2>().norm();
                        interesting_pts_2d_.push_back(Eigen::Vector2d(length * cos(angle), length * sin(angle)));
                    }
                }
                else if (angle_diff > angle_diff_upper_threshold)
                {
                    // the error is too large, which means the two edge points are on the opposite sides of the origin
                    // we consider three directions: the two directions from the origin to the two edge points with additional bias, and the average direction of the two edge points
                    double average_angle = (angle1 + angle2) / 2.0 * (angle2 - angle1) / abs(angle2 - angle1);
                    interesting_pts_2d_.push_back(Eigen::Vector2d(max_distance * cos(angle1 + angle_bias), max_distance * sin(angle1 + angle_bias)));
                    interesting_pts_2d_.push_back(Eigen::Vector2d(max_distance * cos(angle2 - angle_bias), max_distance * sin(angle2 - angle_bias)));
                    interesting_pts_2d_.push_back(Eigen::Vector2d(max_distance * cos(average_angle), max_distance * sin(average_angle)));
                }
                else
                {
                    double average_angle = (angle1 + angle2) / 2.0;
                    interesting_pts_2d_.push_back(Eigen::Vector2d(max_distance * cos(average_angle), max_distance * sin(average_angle)));
                }
            }
        }
    }
}

void InterestingDirectionExtractor::extractDirectionsToObstacles()
{
    // Implement the logic to extract directions to obstacles in 3D
}

void InterestingDirectionExtractor::publishEdgePoints()
{
    visualization_msgs::Marker edge_points_marker;
    if  (env_type_){
        edge_points_marker.header.frame_id = "base_link";
    }
    else{
        edge_points_marker.header.frame_id = "scan";
    }
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

    geometry_msgs::Point p1;

    for (const auto &edge_point : edge_points_with_info_)
    {
        p1.x = edge_point.position[0];
        p1.y = edge_point.position[1];
        p1.z = edge_point.position[2];
        edge_points_marker.points.push_back(p1);
    }
    edge_pts_pub_.publish(edge_points_marker);
}

void InterestingDirectionExtractor::publishInterestingDirections2D()
{
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

    direction_2d_marker.pose.orientation.x = 0.0;
    direction_2d_marker.pose.orientation.y = 0.0;
    direction_2d_marker.pose.orientation.z = 0.0;
    direction_2d_marker.pose.orientation.w = 1.0;

    geometry_msgs::Point p_start, p_end;
    p_start.x = 0.0;
    p_start.y = 0.0;
    p_start.z = 0.0;
    for (const auto &dir : interesting_pts_2d_)
    {
        p_end.x = dir[0];
        p_end.y = dir[1];
        p_end.z = 0.0;
        direction_2d_marker.points.push_back(p_start);
        direction_2d_marker.points.push_back(p_end);
    }
    direction_2d_pub_.publish(direction_2d_marker);
}

void InterestingDirectionExtractor::publishTempMap()
{
    if (temp_map_ == nullptr || temp_map_->points.size() == 0)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_map_with_intensity(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto &pt : temp_map_->points)
    {
        pcl::PointXYZI pt_i;
        pt_i.x = pt.x;
        pt_i.y = pt.y;
        pt_i.z = pt.z;
        pt_i.intensity = 1.0;
        temp_map_with_intensity->points.push_back(pt_i);
    }
    temp_map_with_intensity->width = temp_map_with_intensity->points.size();
    temp_map_with_intensity->height = 1;
    temp_map_with_intensity->is_dense = true;
    temp_map_with_intensity->header.frame_id = "base_link";
    sensor_msgs::PointCloud2 temp_map_msg;
    pcl::toROSMsg(*temp_map_with_intensity, temp_map_msg);
    temp_map_pub_.publish(temp_map_msg);
}

void InterestingDirectionExtractor::publishPoints()
{
    visualization_msgs::Marker tmp_points;

    std::vector<Eigen::Vector3d> debug_points(potential_directions_3d_.direction_to_free_space);
    tmp_points.header.frame_id = "base_link";
    tmp_points.header.stamp = ros::Time::now();
    tmp_points.ns = "debug_points";
    tmp_points.id = 0;
    tmp_points.type = visualization_msgs::Marker::LINE_LIST;
    tmp_points.action = visualization_msgs::Marker::ADD;
    tmp_points.scale.x = 0.05; // shaft diameter

    tmp_points.color.r = 0.0;
    tmp_points.color.g = 0.0;
    tmp_points.color.b = 1.0;
    tmp_points.color.a = 1.0;

    tmp_points.pose.orientation.x = 0.0;
    tmp_points.pose.orientation.y = 0.0;
    tmp_points.pose.orientation.z = 0.0;
    tmp_points.pose.orientation.w = 1.0;

    geometry_msgs::Point p_start, p_end;
    p_start.x = 0.0;
    p_start.y = 0.0;
    p_start.z = 0.0;

    for (const auto &dir : debug_points)
    {
        p_end.x = dir[0];
        p_end.y = dir[1];
        p_end.z = dir[2];
        tmp_points.points.push_back(p_start);
        tmp_points.points.push_back(p_end);
    }

    point_pub_.publish(tmp_points);
}

void InterestingDirectionExtractor::visualizationCallback(const ros::TimerEvent &e)
{
    publishEdgePoints();
    if (!env_type_)
    {
        publishInterestingDirections2D();
    }
    else{
        publishTempMap();
        publishPoints();
    }
}

void InterestingDirectionExtractor::quaternionToRPY(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}