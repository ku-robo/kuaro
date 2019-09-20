#include "stdafx.hpp"

struct Point
{
	double x;
	double y;
	double z;
	double ox;
	double oy;
	double oz;
	double ow;
};

int main(int argc, char **argv)
{
	// ROSの初期化
	ros::init(argc, argv, "waypoint_pub");
	ros::NodeHandle node;

	// パラメータの設定
	ros::NodeHandle ph("~");
	std::string read_file;
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));
	
	
	//変数の宣言
	std::vector<Point> waypoint;

	// publisherの作成
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/waypoint_publisher", 100);

	// whileの回る周期の設定
	ros::Rate rate(10);
	

	// 読み込んだファイルがあるか確認する
	int ret;
	FILE *rfp = fopen(read_file.c_str(), "r");
	if (rfp == NULL)
	{
		ROS_ERROR("Don't exist file!");
		return -1;
	}

	Point tmp;
	while (ret = fscanf(rfp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &tmp.x, &tmp.y, &tmp.z, &tmp.ox, &tmp.oy, &tmp.oz, &tmp.ow) != EOF)
	{
		waypoint.push_back(tmp);
    }
	fclose(rfp);


	while(ros::ok())
	{
		// publishするマーカの設定
		visualization_msgs::Marker points;
		points.header.frame_id = "/map";
		points.header.stamp = ros::Time::now();
		points.ns = "POINTS";
		points.color.b = 1.0;
		points.color.a = 1.0;
		points.scale.x = 0.3;
		points.scale.y = 0.3;
		points.scale.z = 0.3;
		points.action = visualization_msgs::Marker::ADD;
		points.type = visualization_msgs::Marker::POINTS;
		points.pose.orientation.w = 1.0;

		// publishするマーカの座標をwaypointから取得する
		for(int i = 0; i < waypoint.size(); i++)
		{
			geometry_msgs::Point p;
			p.x = waypoint[i].x;
			p.y = waypoint[i].y;
			p.z = 0.25;
			points.points.push_back(p);
		}

		// マーカのパブリッシュ
		marker_pub.publish(points);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
