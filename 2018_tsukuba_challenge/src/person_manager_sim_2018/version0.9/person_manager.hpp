/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/09/05
* <version>		v1.0
*
* <MEMO>
*
* ---------------------------------------------*/

#include "stdafx.hpp"

#pragma once

// move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// mov_baseにゴールを与えるクラス
class PERSON_MANAGER
{
private:

public:
	PERSON_MANAGER();
	~PERSON_MANAGER();

	ros::NodeHandle p_goal_node;

	double goal_point[3];		// 人物までの距離を受け取る変数
	tf::TransformListener listener; // mapとbase_linkのtfを拾うリスナー
	MoveBaseClient ac;			// move_baseのサービスクライアント
	ros::Publisher success_pub; // goalしたことを伝えるパブリッシャー

	void goal_send_callback(const std_msgs::Float32MultiArray::ConstPtr& person_point);
};
