#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

#include <Eigen/Core>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr &);
void crop_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_destination);

ros::Publisher pub;
pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
tf::StampedTransform stamped_transform_base_2_camera;
tf::Transform transform_base_2_camera;

bool is_tf_base_2_camera_exist = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_to_pcd");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, callback);
    tf::TransformListener listener;

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        // Request the tf between ABB base_link frame and the camera frame
        try
        {
            listener.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), stamped_transform_base_2_camera);
            transform_base_2_camera.setOrigin(stamped_transform_base_2_camera.getOrigin());
            transform_base_2_camera.setRotation(stamped_transform_base_2_camera.getRotation());
            is_tf_base_2_camera_exist = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("Error: %s", ex.what());
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (is_tf_base_2_camera_exist)
    {
        pcl_conversions::toPCL(*msg, *cloud_blob);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);

        // Transform the pointcloud to Abb frame
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl_ros::transformPointCloud(*cloud_filtered, *cloud_tf, transform_base_2_camera);
        
        // Crop the point cloud
        // crop_pcl(cloud_tf, cloud_p); 
        
        pcl::PCDWriter writer;
        std::stringstream ss;
        ss << "/home/picko/pickommerce_ws/src/pcl_segmentation/pcd/cloud_sample.pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_filtered, false); //*
        std::cerr << "Saved " << cloud_filtered->size () << " data points to test_pcd.pcd."   << std::endl;

        getchar();

    }
}


void crop_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_destination)
{
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> cropX_axis;
    pcl::PassThrough<pcl::PointXYZRGB> cropY_axis;
    cropX_axis.setInputCloud(cloud_source);
    cropX_axis.setFilterFieldName("x");
    cropX_axis.setFilterLimits(0, 0.9);
    cropX_axis.filter(*cloud_destination);
    cropY_axis.setInputCloud(cloud_destination);
    cropY_axis.setFilterFieldName("y");
    cropY_axis.setFilterLimits(-0.4, 0.3);
    cropY_axis.filter(*cloud_destination);
}