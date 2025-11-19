#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <gazebo_msgs/ModelStates.h>

#include <tf/transform_broadcaster.h>

// Global state: current pose of the robot in Gazebo
geometry_msgs::Pose  g_current_pose;
// Latest command velocity (Vx, Vy, Vz, yaw rate)
geometry_msgs::Twist g_cmd_vel;
// Flag: whether we have received a valid pose from Gazebo
bool g_have_pose = false;

// Publisher for /odom
ros::Publisher g_odom_pub;


// Callback for /gazebo/model_states
// - Update g_current_pose (with normalized quaternion)
// - Publish TF: odom -> base_link
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Static broadcaster: constructed once, reused forever
    static tf::TransformBroadcaster br;
    // Used to avoid TF_REPEATED_DATA warnings
    static ros::Time last_publish_time(0);

    // Use ROS time (with use_sim_time this is Gazebo sim time)
    ros::Time current_time = ros::Time::now();
    ros::Duration publish_interval(0.01); // 100 Hz

    // Limit frequency AND ensure strictly increasing timestamps
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

            // Ensure the quaternion is valid and normalize it
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

            // Write back the normalized quaternion into g_current_pose
            g_current_pose.orientation.x = q.x();
            g_current_pose.orientation.y = q.y();
            g_current_pose.orientation.z = q.z();
            g_current_pose.orientation.w = q.w();

            // Publish the transform from odom to base_link
            transform.setOrigin(tf::Vector3(g_current_pose.position.x,
                                            g_current_pose.position.y,
                                            g_current_pose.position.z));
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform,
                                                  current_time,
                                                  "odom",      // parent frame
                                                  "base_link"  // child frame
                                                  ));

            last_publish_time = current_time;
            g_have_pose = true;  // we now have a valid pose
            break;
        }
    }
}


// Callback for 3D velocity commands (/cmd_vel_3d)
// geometry_msgs/Twist:
//   linear.x, linear.y, linear.z: Vx, Vy, Vz (in base_link frame)
//   angular.z: yaw rate
void cmdVel3DCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    g_cmd_vel = *msg;
}


// Publish odometry using current pose from Gazebo and commanded velocity
void publishOdometry()
{
    // Do not publish odom until we have a valid pose
    if (!g_have_pose)
        return;

    nav_msgs::Odometry odom;
    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // Pose from Gazebo (already normalized quaternion)
    odom.pose.pose = g_current_pose;

    // Use the commanded velocity as twist (we do not estimate real velocities)
    odom.twist.twist = g_cmd_vel;

    g_odom_pub.publish(odom);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_robot_node");
    ros::NodeHandle nh;

    // Subscriber for robot pose from Gazebo
    ros::Subscriber model_states_sub =
        nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    // Subscriber for 3D velocity commands: Vx, Vy, Vz, yaw rate
    ros::Subscriber cmd_vel_3d_sub =
        nh.subscribe("/cmd_vel_3d", 10, cmdVel3DCallback);

    // Odometry publisher
    g_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // Initialize commanded velocity to zero
    g_cmd_vel.linear.x  = 0.0;
    g_cmd_vel.linear.y  = 0.0;
    g_cmd_vel.linear.z  = 0.0;
    g_cmd_vel.angular.x = 0.0;
    g_cmd_vel.angular.y = 0.0;
    g_cmd_vel.angular.z = 0.0;

    ros::Rate loop_rate(100.0); // 100 Hz publish loop

    while (ros::ok())
    {
        ros::spinOnce();

        // Only publish odometry (no SetModelState)
        publishOdometry();

        loop_rate.sleep();
    }

    return 0;
}
