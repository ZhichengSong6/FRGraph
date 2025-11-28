#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>
#include <cmath>  // for sin, cos

geometry_msgs::Pose  g_pose;
geometry_msgs::Twist g_cmd;
bool   g_have_pose = false;

// Our own z controller state (height), independent of Gazebo gravity
double g_z_ctrl = 0.0;

void modelCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "simple_robot")
        {
            g_pose = msg->pose[i];

            // First time we get a pose: initialize z_ctrl from current z
            if (!g_have_pose)
            {
                g_z_ctrl = g_pose.position.z;
            }

            g_have_pose = true;
            break;
        }
    }
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    g_cmd = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_robot_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub_state =
        nh.subscribe("/gazebo/model_states", 5, modelCallback);

    ros::Subscriber sub_cmd =
        nh.subscribe("/cmd_vel", 5, cmdCallback);

    ros::ServiceClient set_state =
        nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    ros::Rate rate(100.0);

    ros::Time last_time = ros::Time::now();

    // Initialize commanded twist to zero
    g_cmd.linear.x  = 0.0;
    g_cmd.linear.y  = 0.0;
    g_cmd.linear.z  = 0.0;
    g_cmd.angular.x = 0.0;
    g_cmd.angular.y = 0.0;
    g_cmd.angular.z = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        ros::Time now = ros::Time::now();
        double dt = (now - last_time).toSec();
        last_time = now;

        if (dt <= 0.0)
        {
            rate.sleep();
            continue;
        }

        if (dt > 0.05) 
        {
            dt = 0.05;
        }

        if (!g_have_pose)
        {
            rate.sleep();
            continue;
        }

        // Extract yaw from quaternion
        tf::Quaternion q(
            g_pose.orientation.x,
            g_pose.orientation.y,
            g_pose.orientation.z,
            g_pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Integrate motion in base_link frame
        double vx = g_cmd.linear.x;
        double vy = g_cmd.linear.y;
        double vz = g_cmd.linear.z;
        double wz = g_cmd.angular.z;

        // Convert (vx, vy) from body frame to world frame
        double dx = (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
        double dy = (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;

        // Our own z integration, independent of gravity
        g_z_ctrl += vz * dt;

        double dyaw = wz * dt;

        // Build new yaw
        yaw += dyaw;
        tf::Quaternion q_new;
        q_new.setRPY(0, 0, yaw);

        // Target pose
        geometry_msgs::Pose new_pose = g_pose;
        new_pose.position.x += dx;
        new_pose.position.y += dy;
        new_pose.position.z  = g_z_ctrl;  // override z with our integrated height

        new_pose.orientation.x = q_new.x();
        new_pose.orientation.y = q_new.y();
        new_pose.orientation.z = q_new.z();
        new_pose.orientation.w = q_new.w();

        // Prepare srv
        gazebo_msgs::SetModelState srv;
        srv.request.model_state.model_name      = "simple_robot";
        srv.request.model_state.pose            = new_pose;
        srv.request.model_state.reference_frame = "world";

        if (!set_state.call(srv))
        {
            ROS_WARN_THROTTLE(1.0, "SetModelState call failed");
        }
        else
        {
            // Update stored pose to the commanded one
            g_pose = new_pose;
        }

        rate.sleep();
    }

    return 0;
}
