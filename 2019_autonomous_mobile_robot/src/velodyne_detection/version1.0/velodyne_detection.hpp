#include "stdafx.hpp"

#pragma once

struct point_xyz
{
	float x;
	float y;
	float z;
};

class C_person_detect
{
public:
	C_person_detect(void);
	~C_person_detect(void);

	void person_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const yolo2::ImageDetections::ConstPtr& msg);
	float median(std::vector<float> v);

private:

	// パラメータ
	float velo_height;
	int TmpWidth;
	int TmpHeight;
	float voxel_size;
	float seg_point_min;
	float seg_point_max;
	float seg_distance;

	ros::NodeHandle pub_node;

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	std::vector<pcl::PointIndices> clusters;

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr view_cloud;
	
	std::vector<point_xyz> median_point;
	std::vector<double> cluster_deg;

  ros::Publisher person_pub;
};
