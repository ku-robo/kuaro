/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/31
* <version>		v2.2
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include <2018_tsukuba_challenge/stdafx.hpp>


struct GetPoint
{
	double position[3];
	double orientation[4];
};


class C_fake_obstacle
{
public:
	C_fake_obstacle(void);
	~C_fake_obstacle(void);

	void get_click_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point);
	void run();

private:
	ros::NodeHandle node;
	ros::Subscriber click_sub;
	ros::Publisher cloud_pub;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	GetPoint click_point;

	bool sub_flag;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud;
};
