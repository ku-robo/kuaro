/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/24
* <version>		v1.3
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "stdafx.hpp"
#include "person_manager.hpp"

C_person_manager::C_person_manager(void)
{
	// launchファイルで設定するパラメータ
	ros::NodeHandle ph("~");
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));
	ph.param("distance", distance, 2.0);
	ph.param("time_limit", time_limit, 5.0);
	ph.param("back_vel", back_vel, -0.1);
	ph.param("start_point", start_point, 0);
	ph.param("map_flag", map_flag, -1);
	std::cout << "read: " << read_file << std::endl;
	std::cout << "distance: " << distance << std::endl;
	std::cout << "time_limit: " << time_limit << std::endl;
	std::cout << "back_vel: " << back_vel << std::endl;
	std::cout << "start_point: " << start_point << std::endl;
	std::cout << "map_flag: " << map_flag << std::endl;

	// waypointが書かれているファイルが存在するか調べる
	FILE* fp;
	fp = fopen(read_file.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("file path error!");
		return;
	}

	// ファイルからゴールを読みとり，プッシュバックしていく
	GetPoint tmp;
	while(fscanf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &tmp.position[0], &tmp.position[1], &tmp.position[2], &tmp.orientation[0], &tmp.orientation[1], &tmp.orientation[2], &tmp.orientation[3]) != EOF)
	{
		goal_point.push_back(tmp);
	}

	cmd_pub = cmd_node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	person_sub = perosn_node.subscribe("/clicked_point", 1, get_person_callback);

}

C_person_manager::~C_person_manager(void)
{

}

double C_person_manager::rounding(double angle)
{
	// 角度を-PI〜PIに丸め込む関数_sato
	while(angle > M_PI)
	{
		angle = angle - 2*M_PI;
	}
	while(angle < -M_PI)
	{
		angle = angle + 2*M_PI;
	}

	return angle;
}


void C_person_manager::get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point)
{
	// サブスクライバのデータを受け取る．　z軸の座標はヨー角を入れている（めんどくさい）
	now_person.position[0] = sub_point->point.x;
	now_person.position[1] = sub_point->point.y;
	now_person.position[2] = sub_point->point.z;
	now_person.orientation[0] = 0.0;
	now_person.orientation[1] = 0.0;
	now_person.orientation[2] = 0.0;
	now_person.orientation[3] = 1.0;
	person_time = sub_point->header.stamp;
	person_point.push_back(now_person);
	person_flag = true;
}


void C_person_manager::run()
{
	ros::Time start_time, current_time;
	ros::Rate rate(30);

	int print_count = 0;
	person_flag = false;
	goal_index = -1;

	// move_baseのクライアントを作成する
	MoveBaseClient ac("move_base", true);

	// クライアント・サービスが立ち上がるまで待機する
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	std::cout << "READY" << std::endl;

	for(int i = start_point; i < goal_point.size(); i++)
	{
		goal_index = i;

		move_base_msgs::MoveBaseGoal goal;

		// min_obstacle distを動的に変更している
		if(map_flag == 0)
		{
			if(i == 0)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

			if(i == 26)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.1");
			}

			if(i == 36)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

			if(i == 45)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.05");
			}

			if(i == 59)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}
		}

		if(map_flag == 1)
		{
			if(i == 0)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

			if(i == 28)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.1");
			}

			if(i == 43)//39
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

			if(i == 53)//47
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.01");
			}

			if(i == 71)//51
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.5");
			}

			// 自動ドアは No.51 から i = 51 で始める
		}

		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = goal_point[i].position[0];
		goal.target_pose.pose.position.y = goal_point[i].position[1];
		goal.target_pose.pose.position.z = goal_point[i].position[2];
		goal.target_pose.pose.orientation.x = goal_point[i].orientation[0];
		goal.target_pose.pose.orientation.y = goal_point[i].orientation[1];
		goal.target_pose.pose.orientation.z = goal_point[i].orientation[2];
		goal.target_pose.pose.orientation.w = goal_point[i].orientation[3];

		printf("%d, %lf, %lf\n", i, goal_point[i].position[0], goal_point[i].position[1]);
		ac.sendGoal(goal);

		while(ros::ok())
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			//std::cout << "state: " << state.toString() << std::endl;

			// 人物を見つけていない場合の通常処理
			if(person_flag == false)
			{
				while(1)
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

				double wx = goal_point[i].position[0];
				double wy = goal_point[i].position[1];
				double rx = transform.getOrigin().x();
				double ry = transform.getOrigin().y();
				double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));

				if(dist < distance)
				{
					ac.cancelAllGoals();
					break;
				}

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && i != (goal_point.size() - 1))
				{
					ROS_INFO("sending...");
					ac.sendGoal(goal);
				}
				else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					ROS_ERROR("ABOoooooooRTED");

					ac.cancelAllGoals();

					print_count = 0;
					start_time = ros::Time::now();
					while (1)
					{
						if(print_count % 1000000 == 0)
						std::cout << "back now !! " << std::endl;

						geometry_msgs::Twist cmd_vel;
						cmd_vel.linear.x = back_vel;
						cmd_vel.linear.y = 0.0;
						cmd_vel.linear.z = 0.0;
						cmd_vel.angular.x = 0.0;
						cmd_vel.angular.y = 0.0;
						cmd_vel.angular.z = 0.0;
						cmd_pub.publish(cmd_vel);
						current_time = ros::Time::now();
						double dt = (current_time - start_time).toSec();
						if (dt > time_limit) break;

						rate.sleep();
						ros::spinOnce();
					}
					ac.sendGoal(goal); //2017_08_31 akimoto
				}
				else if(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE && ac.getState() != actionlib::SimpleClientGoalState::PENDING)
				{
					ROS_INFO("No ACTIVE send goal again");
					ac.sendGoal(goal);
				}
			}


}
