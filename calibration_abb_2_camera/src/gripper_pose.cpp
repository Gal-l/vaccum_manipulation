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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_pose");
    ros::NodeHandle nh;
    tf::TransformListener listener;
    static tf::TransformBroadcaster br;

    ros::Rate loop_rate(4);

    while (nh.ok())
    {
        // Request the tf between ABB base_link frame and the marker_frame
        tf::StampedTransform tf;
        try
        {
            listener.lookupTransform("/base_link", "/link_6", ros::Time(0), tf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Error: %s", ex.what());
            ros::Duration(1.0).sleep();
        }

        ROS_INFO("tf gripper: (%f, %f, %f), %f, %f, %f, %f", tf.getOrigin().getX(),
                                                                     tf.getOrigin().getY(),
                                                                     tf.getOrigin().getZ(),
                                                                     tf.getRotation().getX(),
                                                                     tf.getRotation().getY(),
                                                                     tf.getRotation().getZ(),
                                                                     tf.getRotation().getW());

        ros::spinOnce();
        loop_rate.sleep();
    }
}
