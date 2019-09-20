#include "stdafx.hpp"
#include "velodyne_detection.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_data_get");

	C_person_detect person_detect;

	ros::NodeHandle person_node;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, yolo2::ImageDetections> PersonSyncPolicy;

	message_filters::Subscriber<sensor_msgs::Image> *image_sub (new message_filters::Subscriber<sensor_msgs::Image>(person_node, "/image_raw", 10));
	message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub (new message_filters::Subscriber<sensor_msgs::PointCloud2>(person_node, "/velodyne_points", 10));
	message_filters::Subscriber<yolo2::ImageDetections> *yolo_sub (new message_filters::Subscriber<yolo2::ImageDetections>(person_node, "/yolo2/detections", 10));

	message_filters::Synchronizer<PersonSyncPolicy> *person_sync (new message_filters::Synchronizer<PersonSyncPolicy>(PersonSyncPolicy(10), *image_sub, *cloud_sub, *yolo_sub));

	person_sync->registerCallback(boost::bind(&C_person_detect::person_callback, &person_detect, _1, _2, _3));

	ros::spin();

	return 0;
}
