#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

geometry_msgs::Pose current_pose;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    static ros::Time last_publish_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();
    ros::Duration publish_interval(0.01); // 100 Hz

    if ((current_time - last_publish_time) < publish_interval) {
        return;
    }

    tf::Transform transform;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "simple_robot") {
            current_pose = msg->pose[i];

            // Ensure the quaternion is valid
            tf::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, 
                             current_pose.orientation.z, current_pose.orientation.w);
            if (q.length2() < 1e-6) {
                ROS_WARN("Received invalid quaternion, resetting to identity.");
                q.setRPY(0, 0, 0);
            }
            q.normalize();

            // Publish the transform from base_link to odom
            transform.setOrigin(tf::Vector3(current_pose.position.x, current_pose.position.y, current_pose.position.z));
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
            last_publish_time = current_time;
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_robot");
    ros::NodeHandle nh;

    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 1000, modelStatesCallback);

    ros::spin();
    return 0;
}