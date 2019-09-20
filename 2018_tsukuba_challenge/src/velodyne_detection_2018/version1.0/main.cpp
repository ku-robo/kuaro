#include "stdafx.hpp"
#include "velodyne_detection.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, yolo2::ImageDetections> YoloSyncPolicy;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_data_get");

	C_person_detect person_detect;

	ros::NodeHandle person_node;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub (new message_filters::Subscriber<sensor_msgs::PointCloud2>(person_node, "/velodyne_points", 10));
	message_filters::Subscriber<yolo2::ImageDetections> *yolo_sub (new message_filters::Subscriber<yolo2::ImageDetections>(person_node, "/yolo2/detections", 10));
	
	message_filters::Synchronizer<YoloSyncPolicy> *yolo_sync (new message_filters::Synchronizer<YoloSyncPolicy>(YoloSyncPolicy(10), *cloud_sub, *yolo_sub));
	
	yolo_sync->registerCallback(boost::bind(&C_person_detect::person_callback, &person_detect, _1, _2));

	ros::spin();

	return 0;
}
