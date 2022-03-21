#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"

using namespace std;

// Global variables
tf::StampedTransform transform_camera_2_marker;
bool is_transform_camera_2_marker_exist = false;

// Callback of aruco transform topic. Save the aruco marker transform in global variable.
void aruco_transform(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    transformMsgToTF(msg->transform, transform_camera_2_marker);
    is_transform_camera_2_marker_exist = true; // Notify that we receive the tf message.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/aruco_single/transform", 1, aruco_transform);
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    ros::Rate loop_rate(4);

    while (nh.ok())
    {
        // Request the tf between ABB base_link frame and the marker_frame
        tf::StampedTransform transform_base_2_marker;
        try
        {
            listener.lookupTransform("/base_link", "/marker_frame", ros::Time(0), transform_base_2_marker);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Error: %s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // Initile transform 
        tf::StampedTransform transform_camera_2_base;
        transform_camera_2_base.frame_id_ = "camera_depth_optical_frame";
        transform_camera_2_base.child_frame_id_ = "base_link";
        transform_camera_2_base.stamp_ = ros::Time::now();
        if (is_transform_camera_2_marker_exist)
        {
            transform_camera_2_base.mult(transform_camera_2_marker, transform_base_2_marker.inverse());
            br.sendTransform(transform_camera_2_base);
            ROS_INFO("tf_camera_2_base: %f, %f, %f, %f, %f, %f, %f", transform_camera_2_base.getOrigin().getX(),
                                                                     transform_camera_2_base.getOrigin().getY(),
                                                                     transform_camera_2_base.getOrigin().getZ(),
                                                                     transform_camera_2_base.getRotation().getX(),
                                                                     transform_camera_2_base.getRotation().getY(),
                                                                     transform_camera_2_base.getRotation().getZ(),
                                                                     transform_camera_2_base.getRotation().getW());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
