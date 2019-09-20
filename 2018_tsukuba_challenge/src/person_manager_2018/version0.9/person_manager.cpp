/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/09/05
* <version>		v1.0
*
* <MEMO>
*
* ---------------------------------------------*/

#include "stdafx.hpp"
#include "person_manager.hpp"

// PERSON_MANAGERのコンストラクタ
PERSON_MANAGER::PERSON_MANAGER():
ac("move_base", true)
{
	success_pub = p_goal_node.advertise<std_msgs::String>("goal_state", 100);
}

PERSON_MANAGER::~PERSON_MANAGER()
{

}

// string型のメッセージを受け取って,現在の地点を保存するコールバック関数
void PERSON_MANAGER::goal_send_callback(const std_msgs::Float32MultiArray::ConstPtr& person_point)
{
	std::cout << "PERSON_MANAGER" << std::endl;

	for (int i = 0; i < person_point->data.size(); i++)
		goal_point[i] = (double)person_point->data[i];

	geometry_msgs::PointStamped gm_detect_point;
	geometry_msgs::PointStamped gm_map_point;

	gm_detect_point.header.frame_id = "/base_link";
	gm_detect_point.header.stamp = ros::Time();
	gm_detect_point.point.x = goal_point[0] - 1.0;
	gm_detect_point.point.y = goal_point[1];
	gm_detect_point.point.z = 0.0;
	double ox = 0.0;
	double oy = 0.0;
	double oz = 0.0;
	double ow = 1.0;

	// tfによる地点の取得
	while(1)
	{
		try
		{
			listener.transformPoint("/map", gm_detect_point, gm_map_point);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		break;
	}

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = gm_map_point.point.x;
	goal.target_pose.pose.position.y = gm_map_point.point.y;
	goal.target_pose.pose.position.z = gm_map_point.point.z;
	goal.target_pose.pose.orientation.x = ox;
	goal.target_pose.pose.orientation.y = oy;
	goal.target_pose.pose.orientation.z = oz;
	goal.target_pose.pose.orientation.w = ow;

	ac.sendGoal(goal);

	while(ros::ok())
	{
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		break;
	}

	std::string goal_state = "SUCCEEDED";
	std_msgs::String ros_msg;
	ros_msg.data = goal_state;
	success_pub.publish(ros_msg);
}
