/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/24
* <version>		v1.3
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "stdafx.hpp"

struct GetPoint
{
	double position[3];
	double orientation[4];
};

class C_person_manager
{
public:
	C_person_manager(void);
	~C_person_manager(void);

	double rounding(double angle);
	void get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point);
	void run();

private:
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	ros::NodeHandle cmd_node;
	ros::Publisher cmd_pub;

	ros::NodeHandle perosn_node;
	ros::Subscriber person_sub;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	// launchファイルのパラメータで設定する変数
	std::string read_file;
	double distance;
	double time_limit;
	double back_vel;
	int start_point;
	int map_flag;

	std::vector<GetPoint> goal_point;
	std::vector<GetPoint> person_point;
	GetPoint now_person;
	ros::Time person_time;
	bool person_flag;
	int goal_index;
};



/*-----MAIN--------------------------------------*/
int main(int argc,char *argv[])
{
	ros::init(argc, argv, "person_manager");

	ros::NodeHandle cmd_node;
	ros::Publisher cmd_pub = cmd_node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	ros::NodeHandle perosn_node;
	ros::Subscriber person_sub = perosn_node.subscribe("/clicked_point", 1, get_person_callback);

	// param setting
	ros::NodeHandle ph("~");
	std::string read_file;
	double distance;
	double time_limit;
	double back_vel;
	int start_point;
	int map_flag;
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));
	ph.param("distance", distance, 2.0);
	ph.param("time_limit", time_limit, 5.0);
	ph.param("back_vel", back_vel, -0.1);
	ph.param("start_point", start_point, 0);
	ph.param("map_flag", map_flag, -1);
	printf("read: %s\n", read_file.c_str());
	printf("distance: %lf\n", distance);
	printf("time_limit: %lf\n", time_limit);

	ros::Time start_time, current_time;
	ros::Rate rate(30);
	int print_count = 0;
	person_flag = false;
	goal_index = -1;

	// file open
	FILE* fp;
	fp = fopen(read_file.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("file path error!");
		return -1;
	}

	// goal setting
	GetPoint tmp;
	while(fscanf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &tmp.position[0], &tmp.position[1], &tmp.position[2], &tmp.orientation[0], &tmp.orientation[1], &tmp.orientation[2], &tmp.orientation[3]) != EOF)
	{
		goal_point.push_back(tmp);
	}

	tf::TransformListener listener;
	tf::StampedTransform transform;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	printf("READY!");

	for(int i = start_point; i < goal_point.size(); i++) //i = 18 person
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
			else
			{
				std::cout << "person navigation start" << std::endl;

				ac.cancelAllGoals();
				sleep(1);

				move_base_msgs::MoveBaseGoal person_goal;

				geometry_msgs::PoseStamped gm_base_pose;
				geometry_msgs::PoseStamped gm_map_pose;

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

				double map_dx = now_person.position[0] - transform.getOrigin().x();
				double map_dy = now_person.position[1] - transform.getOrigin().y();

				geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, atan2(map_dy, map_dx));

				person_goal.target_pose.header.frame_id = "/map";
				person_goal.target_pose.header.stamp = ros::Time::now();
				person_goal.target_pose.pose.position.x = now_person.position[0];
				person_goal.target_pose.pose.position.y = now_person.position[1];
				person_goal.target_pose.pose.position.z = 0.0;
				person_goal.target_pose.pose.orientation.x = q.x;
				person_goal.target_pose.pose.orientation.y = q.y;
				person_goal.target_pose.pose.orientation.z = q.z;
				person_goal.target_pose.pose.orientation.w = q.w;

				// 人物の座標に向かう
				ac.sendGoal(person_goal);

				// 人物の座標までナビゲーションを行う． 40秒経過したらナビゲーションを中断する．
				start_time = ros::Time::now();
				print_count = 0;
				while(1)
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

					double rx = transform.getOrigin().x();
					double ry = transform.getOrigin().y();
					double dist = std::sqrt(std::pow(now_person.position[0] - rx, 2) + std::pow(now_person.position[1] - ry, 2));

					if(print_count % 1000000 == 0)
					std::cout << "person state: " << state.toString() << ", dist: " << dist << std::endl;

					if(dist < 1.0)
					{
						std::cout << "person navigation SUCCEEDED !!" << std::endl;
						ac.cancelAllGoals();
						sleep(5);
						break;
					}

					if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						std::cout << "person navigation ABOoooooooRTED !!" << std::endl;
						ac.cancelAllGoals();
						break;
					}

					current_time = ros::Time::now();
					double dt = (current_time - start_time).toSec();
					if (dt > 40.0)
					{
						std::cout << "Time Over !!" << std::endl;
						ac.cancelAllGoals();
						break;
					}

					print_count++;
				}

				sleep(1);

				// 人物までのナビゲーションが終わったら，現在位置から少し下がる．
				start_time = ros::Time::now();
				print_count = 0;
				while (1)
				{
					if(print_count % 1000000 == 0)
					std::cout << "person back now !! " << std::endl;

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
					if (dt > time_limit * 1.5) break;

					print_count++;

					rate.sleep();
					ros::spinOnce();
				}

				// 現在位置を調べる_sato
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

				double robo_x = transform.getOrigin().x();
				double robo_y = transform.getOrigin().y();

				// 次に向かうウェイポイントを決める_sato
				int initial_i = i;
				int waypoint_num_limit = 5;	// いまのウェイポイントからどこまで離れたウェイポイントを見るか
				double th_ono = 0.0;
				double min_waypoint_dist = hypot(goal_point[i].position[0] - robo_x, goal_point[i].position[1] - robo_y); // 最も近いウェイポイントの距離（初期値は人物探索前に行こうとしていたウェイポイント）
				double th_person = atan2(person_goal.target_pose.pose.position.y - robo_y, person_goal.target_pose.pose.position.x - robo_x); // ロボットの位置と人物の位置の角度
				for(int j = i; j < i + waypoint_num_limit ; j++)
				{
					double waypoint_dist = hypot(goal_point[j].position[0] - robo_x, goal_point[j].position[1] - robo_y);	// ウェイポイントの距離
					double th_waypoint = atan2(goal_point[j].position[1] - robo_y, goal_point[j].position[0] - robo_x);		// ロボットの位置とウェイポイントの位置の角度
					double th_person_to_waypoint = rounding(th_waypoint - th_person); 																		// ロボットの位置と人物の位置の角度を基準とした、ロボットの位置とウェイポイントの位置の角度
					std::cout << "j= " << j << "\t" << "dist " << waypoint_dist << std::endl;
					std::cout << "th_waypoint " << th_waypoint << "\t" << "th_person_to_waypoint " << th_person_to_waypoint* 180.0/M_PI << std::endl;
					if (waypoint_dist <= min_waypoint_dist && fabs(th_person_to_waypoint) >= M_PI/4 || j == initial_i)
					{
						min_waypoint_dist = waypoint_dist;
						th_ono = th_person_to_waypoint;
						i = j;
					}
				}

				std::cout << "init goal num " << initial_i <<  std::endl;
				std::cout << "decide goal num " << i << std::endl;

				// 現在位置から向かっていたゴールまでの角度を計算するために，base_link座標系でゴールの座標を取得する．
				gm_map_pose.header.frame_id = "/map";
				gm_map_pose.header.stamp = ros::Time::now();
				gm_map_pose.pose.position.x = goal_point[i].position[0];
				gm_map_pose.pose.position.y = goal_point[i].position[1];
				gm_map_pose.pose.position.z = goal_point[i].position[2];
				gm_map_pose.pose.orientation.x = goal_point[i].orientation[0];
				gm_map_pose.pose.orientation.y = goal_point[i].orientation[1];
				gm_map_pose.pose.orientation.z = goal_point[i].orientation[2];
				gm_map_pose.pose.orientation.w = goal_point[i].orientation[3];

				// map → base_link のtfの実行
				while(1)
				{
					try
					{
						listener.transformPose("/base_link", ros::Time(0), gm_map_pose, "/map", gm_base_pose);
					}
					catch(tf::TransformException& ex)
					{
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}
					break;
				}

				// ゴールまでロボットを向かせる
				double forward_goal_rad = atan2(gm_base_pose.pose.position.y, gm_base_pose.pose.position.x);
				double forward_goal_dist = hypot(gm_base_pose.pose.position.y, gm_base_pose.pose.position.x); //sato
				double rot_need_time = std::abs(forward_goal_rad / 0.2);
				std::cout << "goal_deg1: " << forward_goal_rad*180.0/M_PI << std::endl;
				start_time = ros::Time::now();
				print_count = 0;
				bool rot_flag = true;	// sato
				if(fabs(forward_goal_rad) < M_PI/4)rot_flag = false;	// sato
				while (1)
				{
					if(forward_goal_dist < distance || !rot_flag)break;	// sato
					if(print_count % 1000000 == 0)
					std::cout << "person rotation now !! " << std::endl;

					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = 0.0;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;

					if(forward_goal_rad > 0.0)
					{
						cmd_vel.angular.z = 0.2;
					}
					else
					{
						cmd_vel.angular.z = -0.2;
					}

					cmd_pub.publish(cmd_vel);
					current_time = ros::Time::now();
					double dt = (current_time - start_time).toSec();
					if (dt > rot_need_time) break;

					print_count++;

					rate.sleep();
					ros::spinOnce();
				}

				// ゴールを与え直す_sato ///////////////////////////////////////////////////////////////////////
				goal.target_pose.header.frame_id = "/map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = goal_point[i].position[0];
				goal.target_pose.pose.position.y = goal_point[i].position[1];
				goal.target_pose.pose.position.z = goal_point[i].position[2];
				goal.target_pose.pose.orientation.x = goal_point[i].orientation[0];
				goal.target_pose.pose.orientation.y = goal_point[i].orientation[1];
				goal.target_pose.pose.orientation.z = goal_point[i].orientation[2];
				goal.target_pose.pose.orientation.w = goal_point[i].orientation[3];
				////////////////////////////////////////////////////////////////////////////////////////////

				// ゴールまでロボットのナビゲーションを行う
				while(1)
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

					if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ac.cancelAllGoals();
						break;
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

							print_count++;

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

				sleep(1);

				// 次のゴールまでの角度を計算するために，base_link座標系でゴールの座標を取得する．
				gm_map_pose.header.frame_id = "/map";
				gm_map_pose.header.stamp = ros::Time::now();
				gm_map_pose.pose.position.x = goal_point[i+1].position[0];
				gm_map_pose.pose.position.y = goal_point[i+1].position[1];
				gm_map_pose.pose.position.z = goal_point[i+1].position[2];

				// map → base_link のtfの実行
				while(1)
				{
					try
					{
						listener.transformPose("/base_link", ros::Time(0), gm_map_pose, "/map", gm_base_pose);
					}
					catch(tf::TransformException& ex)
					{
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}
					break;
				}

				// 次のゴールの方をロボットに向かせる
				forward_goal_rad = atan2(gm_base_pose.pose.position.y, gm_base_pose.pose.position.x);
				forward_goal_dist = hypot(gm_base_pose.pose.position.y, gm_base_pose.pose.position.x); //sato
				rot_need_time = std::abs(forward_goal_rad / 0.2);
				std::cout << "goal_deg2: " << forward_goal_rad*180/3.1415 << std::endl;
				start_time = ros::Time::now();
				print_count = 0;
				bool rot_flag2 = true;	//sato
				if(fabs(forward_goal_rad) < M_PI/4)rot_flag2 = false;	//sato
				while (1)
				{
					if(forward_goal_dist < distance || !rot_flag2)break;	//sato
					if(print_count % 1000000 == 0)
					std::cout << "person rotation now !! " << std::endl;

					geometry_msgs::Twist cmd_vel;
					cmd_vel.linear.x = 0.0;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;

					if(forward_goal_rad > 0.0)
					{
						cmd_vel.angular.z = 0.2;
					}
					else
					{
						cmd_vel.angular.z = -0.2;
					}

					cmd_pub.publish(cmd_vel);
					current_time = ros::Time::now();
					double dt = (current_time - start_time).toSec();
					if (dt > rot_need_time) break;

					print_count++;

					rate.sleep();
					ros::spinOnce();
				}

				person_flag = false;
				break;
			}

			rate.sleep(); // 2017_10_24 akimoto
			ros::spinOnce();
		}
	}

	return 0;
}
