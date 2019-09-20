/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/31
* <version>		v2.2
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "fake_obstacle.hpp"

// コンストラクタでパラメータやファイルの読み込みを行う
C_fake_obstacle::C_fake_obstacle(void):
pub_cloud(new pcl::PointCloud<pcl::PointXYZ>())
{
	sub_flag = false;

	// パブリッシャーとサブスクライバの設定
	click_sub = node.subscribe("/clicked_point", 1, &C_fake_obstacle::get_click_callback, this);
	cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/fake_cloud", 10);
}


C_fake_obstacle::~C_fake_obstacle(void)
{

}

// 人物データをここで拾う
void C_fake_obstacle::get_click_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point)
{
	click_point.position[0] = sub_point->point.x;
	click_point.position[1] = sub_point->point.y;
	click_point.position[2] = sub_point->point.z;
	click_point.orientation[0] = 0.0;
	click_point.orientation[1] = 0.0;
	click_point.orientation[2] = 0.0;
	click_point.orientation[3] = 1.0;

	pub_cloud->clear();
	sub_flag = true;
}


// 実質的にプログラムの本体部分
void C_fake_obstacle::run()
{
	ros::Rate main_rate(10);

	while(ros::ok())
	{
		if(sub_flag == false)
		{

		}
		else
		{
			for(int i = 0; i < 1000; i++)
			{
				pcl::PointXYZ point;

				std::random_device seed_gen;
				std::default_random_engine engine(seed_gen());

				// 点の分布は標準偏差で変えること（0.01）
				std::normal_distribution<> norm_x(click_point.position[0], 0.01);
				std::normal_distribution<> norm_y(click_point.position[1], 0.01);
				std::normal_distribution<> norm_z(0.0, 0.1);

				point.x = (float)norm_x(engine);
				point.y = (float)norm_y(engine);
				point.z = (float)norm_z(engine);

				pub_cloud->push_back(point);
			}
			sub_flag = false;
		}

		if(pub_cloud->points.size() > 0.0)
		{
			sensor_msgs::PointCloud2::Ptr cloud_msg (new sensor_msgs::PointCloud2);
			pcl::toROSMsg(*pub_cloud, *cloud_msg);
			cloud_msg->header.frame_id = "/map";
			cloud_msg->header.stamp = ros::Time::now();
			cloud_pub.publish(*cloud_msg);
		}


		main_rate.sleep();
		ros::spinOnce();
	}

	return;
}
