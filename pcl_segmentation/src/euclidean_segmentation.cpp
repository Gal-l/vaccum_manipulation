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

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pickommerce_msgs/ClusterObjectList.h>
#include <pickommerce_msgs/ClusterList.h>
#include <ros/package.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/geometry.h>

#include "yaml-cpp/yaml.h"

void callback(const sensor_msgs::PointCloud2::ConstPtr &);
void crop_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_destination);
pcl::PointCloud<pcl::PointXYZ>::Ptr msg_to_pc(const sensor_msgs::PointCloud2::ConstPtr &msg);
pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf_size);

pcl::SACSegmentation<pcl::PointXYZ> create_segmentation_object();
void pub_cloud(ros::Publisher *pc_pub, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud);
void pub_cloud(ros::Publisher *pc_pub, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr densty_filter_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::vector<pcl::PointIndices> get_cluster_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
 void read_config();
bool get_cluster(pickommerce_msgs::ClusterList::Request &req,pickommerce_msgs::ClusterList::Response &res);

ros::Publisher pc_pub;
ros::Publisher crop_pub;
ros::Publisher all_cluster_pub;
ros::Publisher plane_pub;
 YAML::Node config;

pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
tf::StampedTransform stamped_transform_base_2_camera;
tf::Transform transform_base_2_camera;

bool is_tf_base_2_camera_exist = false;
pickommerce_msgs::ClusterObjectList clsters;

std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>,pcl::PointXYZRGB>> several_megermant_pc;

int counter=0;
bool waiting_for_msg = true;
bool waiting_for_srv = false;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "euclidean_segmentation");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, callback);
  tf::TransformListener listener;
  read_config();
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/euclidean_segmentation/claster_test", 1);
  crop_pub = nh.advertise<sensor_msgs::PointCloud2>("/euclidean_segmentation/crop_test", 1);
  all_cluster_pub = nh.advertise<pickommerce_msgs::ClusterObjectList>("/euclidean_segmentation/clusters", 1);
  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/euclidean_segmentation/removed_plane", 1);


  ros::ServiceServer all_cluster_srv = nh.advertiseService("get_cluster", get_cluster);

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

bool get_cluster(pickommerce_msgs::ClusterList::Request &req,
                pickommerce_msgs::ClusterList::Response &res){
waiting_for_srv=true;

while (!waiting_for_msg) {}

res.data=clsters.data;

waiting_for_srv=false;

return true;
} 

pcl::PointCloud<pcl::PointXYZ>::Ptr msg_to_pc(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  pcl_conversions::toPCL(*msg, *cloud_blob);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered2);

  // Transform the pointcloud to Abb frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(*cloud_filtered2, *cloud_tf, transform_base_2_camera);

  // Crop the point cloud
  crop_pcl(cloud_tf, cloud);

  return cloud;
}

pcl::SACSegmentation<pcl::PointXYZ> create_segmentation_object()
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(config["segmentation_object"]["MaxIterations"].as<int>());
  seg.setDistanceThreshold(config["segmentation_object"]["DistanceThreshold"].as<int>());
  return seg;
}

std::vector<pcl::PointIndices> get_cluster_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setClusterTolerance(config["cluster_indices"]["ClusterTolerance"].as<float>()); // 2cm
  ec.setMinClusterSize(config["cluster_indices"]["MinClusterSize"].as<int>());
  ec.setMaxClusterSize(config["cluster_indices"]["MaxClusterSize"].as<int>());
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  return cluster_indices;
}

void remove_biggest_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg = create_segmentation_object();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  int nr_points = (int)cloud_filtered->size();
  while (cloud_filtered->size() > 0.6 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    pub_cloud(&plane_pub,cloud_plane);

    std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }
}

pcl::PointXYZRGB add_color_to_point(int R,int G,int B, pcl::PointXYZ point){
  
    pcl::PointXYZRGB pt_color;
    pt_color.x = point.x;
    pt_color.y = point.y;
    pt_color.z = point.z;
    pt_color.r = R;
    pt_color.g = G;
    pt_color.b = B;
    return pt_color;
}

void pub_cloud(ros::Publisher *pc_pub, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud)
{
  sensor_msgs::PointCloud2 new_cloud;
  pcl::toROSMsg(*pcl_cloud, new_cloud);
  new_cloud.header.frame_id = "base_link";
  pc_pub->publish(new_cloud);
}

void pub_cloud(ros::Publisher *pc_pub, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
{
  sensor_msgs::PointCloud2 new_cloud;
  pcl::toROSMsg(*pcl_cloud, new_cloud);
  new_cloud.header.frame_id = "base_link";
  pc_pub->publish(new_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB> paint_pc(pcl::PointCloud<pcl::PointXYZRGB> cloud){
  int R = rand() % 255;
  int G = rand() % 255;
  int B = rand() % 255;
  uint32_t rgb = (static_cast<uint32_t>(R) << 16 | static_cast<uint32_t>(G) << 8 | static_cast<uint32_t>(B));
  for(auto &p: cloud.points) p.rgb=rgb;
  return cloud;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr load_cluster_from_pc(std::vector<pcl::PointIndices>::const_iterator it,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto &idx : it->indices)
  {

    // Get Point
    pcl::PointXYZ pt = (*cloud_filtered)[idx];

    pcl::PointXYZRGB pt_color=add_color_to_point(0,0,0,pt);

    cloud_cluster->push_back(pt_color);
  }
  cloud_cluster->width = cloud_cluster->size();
  cloud_cluster->height = 1;
  cloud_cluster->is_dense = true;
  return cloud_cluster;
}

pcl::PointXYZRGB get_avrege_from_2_point(pcl::PointXYZRGB a ,pcl::PointXYZRGB b){
pcl::PointXYZRGB c;
c.x=(a.x+b.x)/2;
c.y=(a.y+b.y)/2;
c.z=(a.z+b.z)/2;
return c;
}

pcl::PointXYZRGB get_centroid_from_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster){

    pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
    for (const auto &point : cloud_cluster->points){
 
      centroid.add(point);
    }

    pcl::PointXYZRGB centroid_point;
    centroid.get (centroid_point);
    centroid_point.r=0;
    centroid_point.g=0;
    centroid_point.b=0;

    return centroid_point;
}

void remove_pop_up_clasters(pickommerce_msgs::ClusterObjectList * clasters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_cluster){



    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGB>,pcl::PointXYZRGB>> sorted_clasters;
    std::vector<int> point_index;

    std::cout <<"several_megermant_pc " << several_megermant_pc.size() <<std::endl;
    
    for (const auto singel_clasters : several_megermant_pc){
    if (sorted_clasters.size()==0){
      sorted_clasters.push_back(singel_clasters);
      point_index.push_back(1);

    }
    else{
      float min_dis=10;
      int index=0;
      for (int i = 0 ; i< sorted_clasters.size(); i ++){
        float tmp_dis  = pcl::geometry::squaredDistance(singel_clasters.second,sorted_clasters[i].second);

        if (tmp_dis<min_dis){
        min_dis=tmp_dis;
        index=i;
        }
      }

      if (min_dis!=0){
        if  (min_dis < config["remove_clster"]["min_dis"].as<float>()){
        sorted_clasters[index]=std::make_pair(sorted_clasters[index].first+singel_clasters.first,
                                          get_avrege_from_2_point(sorted_clasters[index].second,singel_clasters.second));
        point_index[index]+=1;
        }
        else {
            std::cout<<" dis : " << min_dis << std::endl;
            sorted_clasters.push_back(singel_clasters);
            point_index.push_back(1);
        }
      }
      }
      }
        for (int i = 0 ;i < sorted_clasters.size(); ++i){
              if (point_index[i]>config["remove_clster"]["min_clsters"].as<int>()){
                sensor_msgs::PointCloud2 new_cloud;
                pcl::toROSMsg(sorted_clasters[i].first, new_cloud);
                pcl::PointCloud<pcl::PointXYZRGB> painted_pc=paint_pc(sorted_clasters[i].first);
                *all_cloud_cluster+=painted_pc;

                clasters->data.push_back(new_cloud);
              }
         }

         for (const auto index : point_index){
              std::cout<<" index : " << index << std::endl;
         }

         
    several_megermant_pc.clear();
}

void read_config(){
      std::string pkg_path=ros::package::getPath("pcl_segmentation");
      config = YAML::LoadFile(pkg_path+"/config/config.yaml");
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  
  if (is_tf_base_2_camera_exist)
  {

    counter++;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = msg_to_pc(msg);
    pub_cloud(&crop_pub,cloud);

    
    // Create the filtering object: downsample the dataset using a leaf size of 1mm

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = densty_filter_pc(cloud); //filter_pc(cloud, 0.0005);

    pub_cloud(&plane_pub,cloud_filtered);

    //remove_biggest_plane(cloud_filtered   SAC_SAMPLE_SIZE (sample_size_pairs, sample_size_pairs + sizeof (sample_size_pairs) / sizeof (SampleSizeModel)););

    std::vector<pcl::PointIndices> cluster_indices = get_cluster_indices(cloud_filtered);


      

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster = load_cluster_from_pc(it, cloud_filtered);

    pcl::PointXYZRGB cluster_centroid_point = get_centroid_from_pc(cloud_cluster);


    several_megermant_pc.emplace_back(*cloud_cluster, cluster_centroid_point);

    //several_megermant_pc.push_back(*cloud_cluster);


    //pub_cloud(&plane_pub, cluster_new_cloud);

    }
    if (counter%config["remove_clster"]["sampel_num"].as<int>()==0){
      clsters.data.clear();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      remove_pop_up_clasters(&clsters, all_cloud_cluster);

      std::cout<<clsters.data.size()<<std::endl;

      waiting_for_msg=false;
      while (waiting_for_srv){}

      pub_cloud(&pc_pub, filter_pc(all_cloud_cluster,0.01));

      std::cout << "PointCloud representing the Cluster: " << clsters.data.size() << " data points." << std::endl;
      

      waiting_for_msg=true;


    }
  

  }
  
}

void crop_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_destination)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> cropX_axis;
  pcl::PassThrough<pcl::PointXYZ> cropY_axis;
  pcl::PassThrough<pcl::PointXYZ> cropZ_axis;

  cropX_axis.setInputCloud(cloud_source);
  cropX_axis.setFilterFieldName("x");
  cropX_axis.setFilterLimits(config["crop"]["X_min"].as<float>(), config["crop"]["X_max"].as<float>());
  cropX_axis.filter(*cloud_destination);

  cropY_axis.setInputCloud(cloud_destination);
  cropY_axis.setFilterFieldName("y");
  cropY_axis.setFilterLimits(config["crop"]["Y_min"].as<float>(), config["crop"]["Y_max"].as<float>());
  cropY_axis.filter(*cloud_destination);

  cropZ_axis.setInputCloud(cloud_destination);
  cropZ_axis.setFilterFieldName("z");
  cropZ_axis.setFilterLimits(config["crop"]["Z_min"].as<float>(), config["crop"]["Z_max"].as<float>());
  cropZ_axis.filter(*cloud_destination);

  // need to remove beegest plane
  //cropZ_axis.setFilterLimits(0.23, 0.5);

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr densty_filter_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK(config["densty_filter"]["meank"].as<int>());
  sor.setStddevMulThresh (config["densty_filter"]["std_hresh"].as<float>());
  sor.filter (*cloud_filtered);
  return cloud_filtered;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr densty_filter_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK(config["densty_filter"]["meank"].as<int>());
  sor.setStddevMulThresh (config["densty_filter"]["std_hresh"].as<float>());
  sor.filter (*cloud_filtered);
  return cloud_filtered;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf_size){
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  return cloud_filtered;
}