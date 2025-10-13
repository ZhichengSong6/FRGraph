#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// interactive_markers::MenuHandler menu_handler;
ros::Publisher goal_pub;

visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg)
{
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
    visualization_msgs::InteractiveMarkerControl control;

    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    switch (feedback->event_type)
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        {
            ROS_INFO("Marker clicked, publish new goal!");
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.frame_id = "odom"; 
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.pose = feedback->pose;
            goal_pub.publish(goal_msg);
            break;
        }
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            break;
        }
    }
}

void make6DOFMarker(const tf::Vector3 &position){
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "odom"; 
    tf::pointTFToMsg(position, int_marker.pose.position);
    // int_marker.pose.orientation.w = 1;
    // int_marker.pose.orientation.x = 0;
    // int_marker.pose.orientation.y = 0;
    // int_marker.pose.orientation.z = 0;
    int_marker.scale = 0.2;

    int_marker.name = "simple_6DoF";
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeBoxControl(int_marker);

    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;
    control.markers.push_back(makeBox(int_marker));
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    // menu_handler.apply(server, int_marker.name);
    ROS_INFO("Interactive marker server started.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_pose_publisher");
    ros::NodeHandle nh;
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/navigation_goal_3d", 1); 

    server.reset(new interactive_markers::InteractiveMarkerServer("target_pose","",false)); 

    tf::Vector3 position(1.0, 0.0, 0.0);
    make6DOFMarker(position);

    server->applyChanges();

    ros::spin();
    server.reset();
}