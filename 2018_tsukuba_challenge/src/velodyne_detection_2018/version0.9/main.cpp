#include "stdafx.hpp"

const float velo_height = 1.1;

pcl::visualization::CloudViewer viewer("Cloud Viewer");


void velodyne_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr view_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::fromROSMsg(*cloud_msg, *input_cloud);
	
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (input_cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*filtered_cloud);
	
	int cloud_size = (int) filtered_cloud->points.size();
	
	for(int i = 0; i < cloud_size; i++)
	{
		filtered_cloud->points[i].z += velo_height;
	}
	
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (filtered_cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (0.0, 15.0);
	pass.filter (*filtered_cloud);
	
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1, 2.0);
	pass.filter (*filtered_cloud);
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (filtered_cloud);
  
  std::vector<pcl::PointIndices> clusters;  
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  
  ec.setClusterTolerance (0.5);  
  ec.setMinClusterSize (10);  
  ec.setMaxClusterSize (25000);  
  ec.setSearchMethod (tree);  
  ec.setInputCloud(filtered_cloud);  
  ec.extract (clusters);
  
  float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
  for (int i = 0; i < clusters.size(); ++i)
  {
    std::vector<int> cluster_index = clusters[i].indices;
    for (int j = 0; j < cluster_index.size(); ++j)
    {
      pcl::PointXYZRGB point;
      point.x = filtered_cloud->points[cluster_index[j]].x;
      point.y = filtered_cloud->points[cluster_index[j]].y;
      point.z = filtered_cloud->points[cluster_index[j]].z;
      point.r = colors[i%6][0];
      point.g = colors[i%6][1];
      point.b = colors[i%6][2];
      view_cloud->push_back(point);
    }
	}
	
	viewer.showCloud(view_cloud);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "velodyne_sub");
	ros::NodeHandle node;
	
	ros::Subscriber velodyne_cloud_sub = node.subscribe("/velodyne_points", 1, velodyne_cloud_callback);
	
	ros::spin();
	
	return 0;
}
