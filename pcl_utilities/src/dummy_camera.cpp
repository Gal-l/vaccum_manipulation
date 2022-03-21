#include <iostream>
#include <string>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <ros/package.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::init(argc, argv, "dummy_camera");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("/camera/depth/color/points", 1);

    std::string pkg_path = ros::package::getPath("pcl_utilities");
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(pkg_path + "/pcd/world_scene_2.pcd", *cloud);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_depth_optical_frame";

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}