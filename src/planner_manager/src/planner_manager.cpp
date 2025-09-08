#include "planner_manager.h"

void PlannerManager::initPlannerModule(ros::NodeHandle &nh) {
    node_ = nh;

    if(env_type_ == 3){
        ROS_INFO("3D environment, using velodyne pointcloud");
        velodyne_sub_ = node_.subscribe("/velodyne_points", 1, &PlannerManager::velodyneCallback, this);
    }
    else if(env_type_ == 2){
        ROS_INFO("2D environment, using 2D laser scan");
        scan2d_sub_ = node_.subscribe("/scan", 1, &PlannerManager::scan2dCallback, this);
    }
    else{
        ROS_ERROR("Wrong env_type! Please check the param env_type in the config file.");
    }

    lidar_to_base_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
	tf2_ros::Buffer tfBuffer_lidar;
	tf2_ros::TransformListener tfListener_lidar(tfBuffer_lidar);
	try{
		*(lidar_to_base_ptr_) = tfBuffer_lidar.lookupTransform("base_link", "scan", ros::Time::now(), ros::Duration(2.0));
	}
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    const Eigen::Affine3d T_lidar_base = tf2::transformToEigen(*lidar_to_base_ptr_);
    T_lidar_base_mat = T_lidar_base.matrix().cast<float>();

    base_to_odom_ptr_ = geometry_msgs::TransformStamped::Ptr(new geometry_msgs::TransformStamped);
    tf_listener_odom_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_odom_);

    odom_timer_ = node_.createTimer(ros::Duration(0.1), &PlannerManager::odomTimerCallback, this);

    free_regions_graph_ptr_.reset(new FreeRegionsGraph());
}

void PlannerManager::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Process the Velodyne point cloud data
    if (msg->data.size() == 0)
        {
        ROS_WARN("Received empty point cloud data");
        return;
        }
    
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

    vec_Vec3f pointcloud = DecompROS::cloud_to_vec(cloud);
    vec_Vec3f pointcloud_croped;

    // crop the point cloud
    for (const auto &pt : pointcloud) {
        if (pt[0] > -size_of_croped_pointcloud_[0]/2 && pt[0] < size_of_croped_pointcloud_[0]/2 &&
            pt[1] > -size_of_croped_pointcloud_[1]/2 && pt[1] < size_of_croped_pointcloud_[1]/2 &&
            pt[2] > -size_of_croped_pointcloud_[2]/2 && pt[2] < size_of_croped_pointcloud_[2]/2) {
                pointcloud_croped.push_back(pt);
            }
    }

    // pointcloud are in scan frame, transform it into base frame
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    const Eigen::Matrix4f T_base_odom_mat = T_base_odom.matrix().cast<float>(); 

    vec_Vec3f pointcloud_croped_base_frame;
    for (const auto &pt : pointcloud_croped) {
        Eigen::Vector4f pt_homogeneous(pt[0], pt[1], pt[2], 1.0);
        Eigen::Vector4f pt_transformed = T_base_odom_mat * T_lidar_base_mat * pt_homogeneous;
        pointcloud_croped_base_frame.push_back(Eigen::Vector3d(pt_transformed[0], pt_transformed[1], pt_transformed[2]));
    }
    pointcloud_croped_base_frame_ = pointcloud_croped_base_frame;
}

void PlannerManager::scan2dCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    // Process the 2D laser scan data
    if (msg->ranges.size() == 0)
        {
        ROS_WARN("Received empty laser scan data");
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

    vec_Vec2f pointcloud_croped_2d;

    // crop the point cloud
    for (const auto &pt : pointcloud_2d) {
        if (pt[0] > -size_of_croped_pointcloud_[0]/2 && pt[0] < size_of_croped_pointcloud_[0]/2 &&
            pt[1] > -size_of_croped_pointcloud_[1]/2 && pt[1] < size_of_croped_pointcloud_[1]/2) {
                pointcloud_croped_2d.push_back(pt);
            }
    }

    // pointcloud are in scan frame, transform it into base frame
    const Eigen::Affine3d T_base_odom = tf2::transformToEigen(*base_to_odom_ptr_);
    const Eigen::Matrix4f T_base_odom_mat = T_base_odom.matrix().cast<float>();

    vec_Vec2f pointcloud_croped_2d_base_frame;
    for (const auto &pt : pointcloud_croped_2d) {
        Eigen::Vector4f pt_homogeneous(pt[0], pt[1], 0.0, 1.0);
        Eigen::Vector4f pt_transformed = T_base_odom_mat * T_lidar_base_mat * pt_homogeneous;
        pointcloud_croped_2d_base_frame.push_back(Eigen::Vector2d(pt_transformed[0], pt_transformed[1]));
    }
    pointcloud_croped_base_frame_2d_ = pointcloud_croped_2d_base_frame;
}

void PlannerManager::odomTimerCallback(const ros::TimerEvent &event) {
    try{
        *base_to_odom_ptr_ = tfBuffer_odom_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}
