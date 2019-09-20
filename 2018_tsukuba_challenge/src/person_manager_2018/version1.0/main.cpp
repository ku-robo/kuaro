/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/09/05
* <version>		v1.0
*
* <MEMO>
*
* ---------------------------------------------*/

#include "stdafx.hpp"

struct GetPoint
{
	double position[3];
	double orientation[4];
};

// global_val
std::vector<GetPoint> goal_point;
std::vector<GetPoint> person_point;
GetPoint now_person;
ros::Time person_time;
bool person_flag;
int goal_index;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 人物の座標をキャッチするサブスクライバ
void get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point)
{
	// 人物を一人でも見つけたら，以降の処理をやめる
	if(person_point.size() != 0)
	{
		return;
	}

	// 人物がいる区間の waypoint を記録しておく
	if(22 < goal_index && goal_index < 28) // 20-28
	{
		std::cout << "Search Target!" << std::endl;
	}
	else
	{
		return;
	}

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

/*-----MAIN--------------------------------------*/
int main(int argc,char *argv[])
{
	ros::init(argc, argv, "person_manager");

	ros::NodeHandle cmd_node;
	ros::Publisher cmd_pub = cmd_node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	ros::NodeHandle perosn_node;
	ros::Subscriber person_sub = perosn_node.subscribe("/person_point", 1, get_person_callback);

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
				// 人物を見つけた場合の特別な処理

				// 現在向かっているすべてのゴールをキャンセルする
				ac.cancelAllGoals();
				sleep(1);

				move_base_msgs::MoveBaseGoal person_goal;

				// 人物の座標がbase_link座標系で送られてくるので，map座標系に変換する
				geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, now_person.position[2]);
				geometry_msgs::PoseStamped gm_base_pose;
				geometry_msgs::PoseStamped gm_map_pose;
				gm_base_pose.header.frame_id = "/base_link";
				gm_base_pose.header.stamp = person_time;
				gm_base_pose.pose.position.x = now_person.position[0] - (0.5 * cos(now_person.position[2]));
				gm_base_pose.pose.position.y = now_person.position[1] - (0.5 * sin(now_person.position[2]));
				gm_base_pose.pose.position.z = 0.0;
				gm_base_pose.pose.orientation.x = q.x;
				gm_base_pose.pose.orientation.y = q.y;
				gm_base_pose.pose.orientation.z = q.z;
				gm_base_pose.pose.orientation.w = q.w;

				// base_link → map のtfの実行
				while(1)
				{
					try
					{
						listener.transformPose("/map", person_time, gm_base_pose, "/base_link", gm_map_pose);
					}
					catch(tf::TransformException& ex)
					{
						ROS_ERROR("%s",ex.what());
						ros::Duration(1.0).sleep();
						continue;
					}
					break;
				}

				// map座標系で人物の座標を取得した
				person_goal.target_pose.header.frame_id = "/map";
				person_goal.target_pose.header.stamp = ros::Time::now();
				person_goal.target_pose.pose.position.x = gm_map_pose.pose.position.x;
				person_goal.target_pose.pose.position.y = gm_map_pose.pose.position.y;
				person_goal.target_pose.pose.position.z = 0.0;
				person_goal.target_pose.pose.orientation.x = gm_map_pose.pose.orientation.x;
				person_goal.target_pose.pose.orientation.y = gm_map_pose.pose.orientation.y;
				person_goal.target_pose.pose.orientation.z = gm_map_pose.pose.orientation.z;
				person_goal.target_pose.pose.orientation.w = gm_map_pose.pose.orientation.w;

				printf("person, %lf, %lf\n", gm_map_pose.pose.position.x, gm_map_pose.pose.position.y);

				// 人物の座標に向かう
				ac.sendGoal(person_goal);

				// 人物の座標までナビゲーションを行う． 35秒経過したらナビゲーションを中断する．
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
					double dist = std::sqrt(std::pow(gm_map_pose.pose.position.x - rx, 2) + std::pow(gm_map_pose.pose.position.y - ry, 2));

					if(print_count % 1000000 == 0)
					std::cout << "person state: " << state.toString() << ", dist: " << dist << std::endl;

					if(dist < 1.0)
					{
						ac.cancelAllGoals();
						sleep(4);
						break;
					}

					if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						ac.cancelAllGoals();
						break;
					}

					current_time = ros::Time::now();
					double dt = (current_time - start_time).toSec();
					if (dt > 35.0)
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
					ros::spinOnce();
				}

				// 現在位置から向かっていたゴールまでの角度を計算するために，base_link座標系でゴールの座標を取得する．
				gm_map_pose.header.frame_id = "/map";
				gm_map_pose.header.stamp = ros::Time::now();
				gm_map_pose.pose.position.x = goal_point[i].position[0];
				gm_map_pose.pose.position.y = goal_point[i].position[1];
				gm_map_pose.pose.position.z = goal_point[i].position[2];

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
				double rot_need_time = std::abs(forward_goal_rad / 0.2);
				std::cout << "goal_deg1: " << forward_goal_rad*180/3.1415 << std::endl;
				start_time = ros::Time::now();
				print_count = 0;
				while (1)
				{
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
					ros::spinOnce();
				}

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
				rot_need_time = std::abs(forward_goal_rad / 0.2);
				std::cout << "goal_deg2: " << forward_goal_rad*180/3.1415 << std::endl;
				start_time = ros::Time::now();
				print_count = 0;
				while (1)
				{
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
					ros::spinOnce();
				}

				person_flag = false;
				break;
			}

			ros::spinOnce();
		}
	}

	return 0;
}
