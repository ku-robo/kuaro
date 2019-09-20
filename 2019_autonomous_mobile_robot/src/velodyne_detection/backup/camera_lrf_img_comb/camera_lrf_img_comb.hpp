#include "stdafx.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> CameraSyncPolicy;

struct rgb{
    int b;
    int g;
    int r;
};

class C_camera_comb
{
	public:
		C_camera_comb(void);
		~C_camera_comb(void);
    void run(void);

	private:
		rgb pseudo_color(double input_value);
		void camera_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

		ros::NodeHandle node;
    int queue_size_aki;
		std::string img_topic;
		std::string cloud_topic;

		sensor_msgs::PointCloud2 cloud_tf;

		tf::TransformListener listener;
		message_filters::Subscriber<sensor_msgs::Image> *image_sub;
		message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
		message_filters::Synchronizer<CameraSyncPolicy> *camera_sync;
};
