#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Global state: current pose and twist of the robot in Gazebo
geometry_msgs::Pose  g_current_pose;
geometry_msgs::Twist g_current_twist_world;

// Latest command velocity (kept only if you still want to monitor it;
// no longer used to fake odometry twist)
geometry_msgs::Twist g_cmd_vel;

// Flag: whether we have received a valid pose/twist from Gazebo
bool g_have_pose = false;

// Publisher for /odom
ros::Publisher g_odom_pub;


// Callback for /gazebo/model_states
// - Update g_current_pose and g_current_twist_world
// - Publish TF: odom -> base_link
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    static ros::Time last_publish_time(0);

    ros::Time current_time = ros::Time::now();
    ros::Duration publish_interval(0.01); // 100 Hz

    if ((current_time - last_publish_time) < publish_interval ||
        current_time <= last_publish_time)
    {
        return;
    }

    tf::Transform transform;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "simple_robot")
        {
            g_current_pose = msg->pose[i];
            g_current_twist_world = msg->twist[i];

            // Normalize quaternion
            tf::Quaternion q(g_current_pose.orientation.x,
                             g_current_pose.orientation.y,
                             g_current_pose.orientation.z,
                             g_current_pose.orientation.w);

            if (q.length2() < 1e-6)
            {
                ROS_WARN_THROTTLE(1.0, "Received invalid quaternion from Gazebo, resetting to identity.");
                q.setRPY(0, 0, 0);
            }
            q.normalize();

            g_current_pose.orientation.x = q.x();
            g_current_pose.orientation.y = q.y();
            g_current_pose.orientation.z = q.z();
            g_current_pose.orientation.w = q.w();

            // Publish TF: odom -> base_link
            transform.setOrigin(tf::Vector3(g_current_pose.position.x,
                                            g_current_pose.position.y,
                                            g_current_pose.position.z));
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform,
                                                  current_time,
                                                  "odom",
                                                  "base_link"));

            last_publish_time = current_time;
            g_have_pose = true;
            break;
        }
    }
}


// Callback for velocity commands (/cmd_vel)
// Kept only for completeness/debug; no longer used as odom twist.
void cmdVel3DCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    g_cmd_vel = *msg;
}


// Publish odometry using current pose/twist from Gazebo
void publishOdometry()
{
    if (!g_have_pose)
        return;

    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // Pose from Gazebo
    odom.pose.pose = g_current_pose;

    // Convert Gazebo/world-frame twist to base_link-frame twist
    tf::Quaternion q(g_current_pose.orientation.x,
                     g_current_pose.orientation.y,
                     g_current_pose.orientation.z,
                     g_current_pose.orientation.w);
    q.normalize();

    tf::Matrix3x3 R_wb(q);
    tf::Vector3 v_w(g_current_twist_world.linear.x,
                    g_current_twist_world.linear.y,
                    g_current_twist_world.linear.z);
    tf::Vector3 w_w(g_current_twist_world.angular.x,
                    g_current_twist_world.angular.y,
                    g_current_twist_world.angular.z);

    // base <- world
    tf::Matrix3x3 R_bw = R_wb.transpose();
    tf::Vector3 v_b = R_bw * v_w;
    tf::Vector3 w_b = R_bw * w_w;

    odom.twist.twist.linear.x  = v_b.x();
    odom.twist.twist.linear.y  = v_b.y();
    odom.twist.twist.linear.z  = v_b.z();
    odom.twist.twist.angular.x = w_b.x();
    odom.twist.twist.angular.y = w_b.y();
    odom.twist.twist.angular.z = w_b.z();

    g_odom_pub.publish(odom);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_robot_node");
    ros::NodeHandle nh;

    ros::Subscriber model_states_sub =
        nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    ros::Subscriber cmd_vel_3d_sub =
        nh.subscribe("/cmd_vel", 10, cmdVel3DCallback);

    g_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    g_cmd_vel.linear.x  = 0.0;
    g_cmd_vel.linear.y  = 0.0;
    g_cmd_vel.linear.z  = 0.0;
    g_cmd_vel.angular.x = 0.0;
    g_cmd_vel.angular.y = 0.0;
    g_cmd_vel.angular.z = 0.0;

    ros::Rate loop_rate(100.0);

    while (ros::ok())
    {
        ros::spinOnce();
        publishOdometry();
        loop_rate.sleep();
    }

    return 0;
}