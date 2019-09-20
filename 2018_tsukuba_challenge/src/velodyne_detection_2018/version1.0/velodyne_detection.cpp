#include "stdafx.hpp"
#include "velodyne_detection.hpp"

pcl::visualization::CloudViewer viewer("Cloud Viewer");


C_person_detect::C_person_detect(void):
input_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
view_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
tree((new pcl::search::KdTree<pcl::PointXYZ>()))
{
	ros::NodeHandle private_nh("~");
	private_nh.param<float>("velodyne_height", velo_height, 1.1);
	private_nh.param<int>("TmpWidth", TmpWidth, 640);
	private_nh.param<int>("TmpHeight", TmpHeight, 480);
	private_nh.param<float>("voxel_size", voxel_size, 1.0);
	private_nh.param<float>("seg_point_min", seg_point_min, 5.0);
	private_nh.param<float>("seg_point_max", seg_point_max, 1000.0);
	private_nh.param<float>("seg_distance", seg_distance, 0.5);

	vg.setLeafSize(voxel_size, voxel_size, voxel_size);

	ec.setMinClusterSize(seg_point_min);
	ec.setMaxClusterSize(seg_point_max);
	ec.setClusterTolerance (seg_distance);

	person_pub = pub_node.advertise<geometry_msgs::PointStamped>("/person_point", 100);
}

C_person_detect::~C_person_detect(void)
{

}

float C_person_detect::median(std::vector<float> v)
{
	int size = v.size();
	std::sort(v.begin(), v.end(), std::greater<float>() );

	if (size % 2 == 1)
	{
		return v[(size - 1) / 2];
	}
	else
	{
		return (v[(size / 2) - 1] + v[size / 2]) * 0.5;
	}
}

void C_person_detect::person_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const yolo2::ImageDetections::ConstPtr& msg)
{
	std::cout << "--------------------------------------------------------" << std::endl;

	ros::Time cloud_time = cloud_msg->header.stamp;
	ros::Time yolo_time = msg->header.stamp;

	std::cout << "velodyne: " << cloud_time.sec << ", " << "YOLO: " << yolo_time.sec << std::endl;

	bool detect_flag = false;
	int person_index = -1;
	int class_id = -1;
	float colors[6][3] ={{255,0,0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};

	double yolo_x = 0.0;
	double yolo_f = 620.119;
	for (int i = 0; i<msg->detections.size(); ++i)
	{
		const yolo2::Detection &data = msg->detections[i];
		class_id = data.class_id;
		if(class_id == 0)
		{
			yolo_x = data.x*TmpWidth;
		}
	}
	yolo_x = -(yolo_x - (double)(TmpWidth/2.0));
	double yolo_rad = atan2(yolo_x, yolo_f);
	double yolo_deg = atan2(yolo_x, yolo_f) * 180.0 / M_PI;

	if(yolo_x != (double)(TmpWidth/2.0))
	{
		//std::cout << "No.1" << std::endl;

		pcl::fromROSMsg(*cloud_msg, *input_cloud);

		vg.setInputCloud (input_cloud);
		vg.filter (*filtered_cloud);

		int cloud_size = (int) filtered_cloud->points.size();

		for(int i = 0; i < cloud_size; i++)
		{
			filtered_cloud->points[i].z += velo_height;
		}

		//std::cout << "No.2" << std::endl;

		pass.setInputCloud (filtered_cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (0.0, 10.0);
		pass.filter (*filtered_cloud);

		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.3, 1.6);
		pass.filter (*filtered_cloud);

		//std::cout << "No.3" << std::endl;

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (filtered_cloud);

		//std::cout << "No.4" << std::endl;

		ec.setSearchMethod (tree);
		ec.setInputCloud(filtered_cloud);
		ec.extract (clusters);

		if((int)clusters.size() == 0)
			return;

		//std::cout << "No.5" << std::endl;

		std::vector<float> cluster_x_indices;
		std::vector<float> cluster_y_indices;
		std::vector<float> cluster_z_indices;

		//std::cout << "No.6" << std::endl;

		median_point.resize((int)clusters.size());
		cluster_deg.resize((int)clusters.size());

		//std::cout << "No.7" << std::endl;

		std::cout << "" << std::endl;
		for (int i = 0; i < clusters.size(); ++i)
		{
			std::vector<int> cluster_index = clusters[i].indices;
			for (int j = 0; j < cluster_index.size(); ++j)
			{
				cluster_x_indices.push_back(filtered_cloud->points[cluster_index[j]].x);
				cluster_y_indices.push_back(filtered_cloud->points[cluster_index[j]].y);
				cluster_z_indices.push_back(filtered_cloud->points[cluster_index[j]].z);
			}

			//std::cout << "No.8" << std::endl;

			float median_x = median(cluster_x_indices);
			float median_y = median(cluster_y_indices);
			float median_z = median(cluster_z_indices);

			point_xyz tmp_median;
			tmp_median.x = median_x;
			tmp_median.y = median_y;
			tmp_median.z = median_z;
			median_point[i] = tmp_median;

			cluster_deg[i] = atan2(median_y, median_x) * 180.0 / M_PI;

			//std::cout << "cluster No." << i << ": " << cluster_deg[i] << " deg" << " ( " << median_x << ", " << median_y << " ) " << std::endl;

			cluster_x_indices.clear();
			cluster_y_indices.clear();
			cluster_z_indices.clear();
		}

		//std::cout << "No.9" << std::endl;

		double diff_deg = 10000.0;
		for (int i = 0; i < clusters.size(); ++i)
		{
			double tmp_diff = std::abs(cluster_deg[i] - yolo_deg);
			if(diff_deg > tmp_diff && 3.0 > tmp_diff)
			{
				diff_deg = tmp_diff;
				person_index = i;
			}
		}

		if(class_id == 0 && person_index > 0)
		{
			// 人物座標のパブリッシュ
			geometry_msgs::PointStamped person_point;
			person_point.header.frame_id = "/base_link";
			person_point.header.stamp = yolo_time;
			person_point.point.x = median_point[person_index].x;
			person_point.point.y = median_point[person_index].y;
			person_point.point.z = yolo_rad;
			person_pub.publish(person_point);
			detect_flag = true;
		}

		if(detect_flag)
		{
			for (int i = 0; i < clusters.size(); ++i)
		  {
				std::vector<int> cluster_index = clusters[i].indices;
				for (int j = 0; j < cluster_index.size(); ++j)
		    {
					if(i == person_index)
					{
						pcl::PointXYZRGB point;
						point.x = filtered_cloud->points[cluster_index[j]].x;
						point.y = filtered_cloud->points[cluster_index[j]].y;
						point.z = filtered_cloud->points[cluster_index[j]].z;
						point.r = 255;
						point.g = 0;
						point.b = 0;
						view_cloud->push_back(point);
					}
					else
					{
						pcl::PointXYZRGB point;
						point.x = filtered_cloud->points[cluster_index[j]].x;
						point.y = filtered_cloud->points[cluster_index[j]].y;
						point.z = filtered_cloud->points[cluster_index[j]].z;
						point.r = 0;
						point.g = 0;
						point.b = 255;
						view_cloud->push_back(point);
					}

				}
			}

			std::cout << "Person" << ": " << cluster_deg[person_index]*180.0/3.1415 << " deg" << std::endl;
		}
		else
		{
			// for (int i = 0; i < clusters.size(); ++i)
			// {
			// 	std::vector<int> cluster_index = clusters[i].indices;
			// 	for (int j = 0; j < cluster_index.size(); ++j)
			// 	{
			// 		pcl::PointXYZRGB point;
			// 		point.x = filtered_cloud->points[cluster_index[j]].x;
			// 		point.y = filtered_cloud->points[cluster_index[j]].y;
			// 		point.z = filtered_cloud->points[cluster_index[j]].z;
			// 		point.r = colors[i%6][0];
			// 		point.g = colors[i%6][1];
			// 		point.b = colors[i%6][2];
			// 		view_cloud->push_back(point);
			// 	}
			// }
		}

		std::cout << "YOLO" << ": " << yolo_deg << " deg" << std::endl;
	}
	else
	{
		std::cout << "YOLO OUT" << std::endl;
	}

	std::cout << "" << std::endl;




	viewer.showCloud(view_cloud);

	if(detect_flag)
	{
		std::cout << "detectttttttttttttttttttttttttttt" << std::endl;
		//sleep(1000);
	}

	input_cloud->clear();
	filtered_cloud->clear();
	view_cloud->clear();
	clusters.clear();
}
