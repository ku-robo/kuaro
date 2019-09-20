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
	ros::init(argc, argv, "pose_subscribe");
	ros::NodeHandle node;

	//変数の宣言
	int point_count = 0;
	std::vector<Point> waypoint;

	// 一番最初の点
	Point init_point;
	init_point.x = 0.0;
	init_point.y = 0.0;
	init_point.z = 0.0;
	init_point.ox = 0.0;
	init_point.oy = 0.0;
	init_point.oz = 0.0;
	init_point.ow = 1,0;
	waypoint.push_back(init_point);

	// パラメータの読みこみ
	ros::NodeHandle ph("~");
	std::string read_file_path;
	double waypoint_distance;
	ph.param("read_file_path", read_file_path, std::string("/home/useful/waypoint.txt"));
	ph.param("waypoint_distance", waypoint_distance, 3.0);

	// 読み込んだファイルがあるか確認する
	std::ofstream ofs;
	ofs.open(read_file_path.c_str() , std::ios::out);
	if (ofs.fail())
	{
		std::cerr << "waypointのファイルが見つかりません" << std::endl;
		return -1;
	}

	// tfのListenerとTransformの作成
	tf::TransformListener listener;
	tf::StampedTransform transform;

	// publisherの作成
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/waypoint_marker", 100);

	// whileの回る周期の設定
	ros::Rate rate(10);

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

		// tfを利用して /map上の座標を取得する
		while(ros::ok())
		{
			try
			{
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			break;
		}

		// tfの値を一時保存の変数に格納しておく
		Point tmp_point;
		tmp_point.x = transform.getOrigin().x();
		tmp_point.y = transform.getOrigin().y();
		tmp_point.z = transform.getOrigin().z();
		tmp_point.ox = transform.getRotation().x();
		tmp_point.oy = transform.getRotation().y();
		tmp_point.oz = transform.getRotation().z();
		tmp_point.ow = transform.getRotation().w();

		// 現在位置と最後に置いたwaypointを比較する
		double dist_x = tmp_point.x - waypoint[point_count].x;
		double dist_y = tmp_point.y - waypoint[point_count].y;
		double dist = std::sqrt(std::pow(dist_x, 2) + std::pow(dist_y, 2));

		// 距離が閾値以上だったらwaypointに追加する
		if(dist > waypoint_distance)
		{
			std::cout << "waypoint NO." << point_count << " " << waypoint[point_count].x << ", " << waypoint[point_count].y << std::endl;

			waypoint.push_back(tmp_point);
			point_count++;
		}

		//std::cout << waypoint.size() << std::endl;

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

	// ファイル書き込み i=0は最初の地点なので無視する
	for(int i = 1; i < waypoint.size(); i++)
	{
		ofs << waypoint[i].x <<","<< waypoint[i].y << "," << waypoint[i].z << ","
		<< waypoint[i].ox << "," << waypoint[i].oy << "," << waypoint[i].oz << ","
		<< waypoint[i].ow << std::endl;
	}

	return 0;
}
