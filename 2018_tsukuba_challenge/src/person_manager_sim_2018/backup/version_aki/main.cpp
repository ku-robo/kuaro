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

void get_person_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	now_person.position[0] = msg->point.x;
	now_person.position[1] = msg->point.y;
	now_person.position[2] = msg->point.z;
	now_person.orientation[0] = 0.0;
	now_person.orientation[1] = 0.0;
	now_person.orientation[2] = 0.0;
	now_person.orientation[3] = 0.0;
	person_time = msg->header.stamp;
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
	ros::Subscriber person_sub = perosn_node.subscribe("/clicked_point", 1, get_person_callback);

	// param setting
	ros::NodeHandle ph("~");
	std::string read_file;
	double distance;
	double time_limit;
	double back_vel;
	int map_flag;
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));
	ph.param("distance", distance, 2.0);
	ph.param("time_limit", time_limit, 5.0);
	ph.param("back_vel", back_vel, -0.1);
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

	ros::Rate rate(10);

	for(int i = 0; i < goal_point.size(); i++) //i = 18 person
	{
		goal_index = i;

		move_base_msgs::MoveBaseGoal goal;

		if(map_flag == 0)
		{
			if(i == 0)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

			if(i == 24)
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.1");
			}

			if(i == 36)
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

			if(i == 28) //28
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.5");
			}

			if(i == 43)//43, 39
			{
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");
			}

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

				ac.sendGoal(person_goal);

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

				std::cout << "person navigation finish" << std::endl;

				sleep(1);

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

				std::cout << "person back finish" << std::endl;

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

				tf::Matrix3x3 m_now(transform.getRotation());
				double roll_now, pitch_now, yaw_now;
				m_now.getRPY(roll_now, pitch_now, yaw_now);

				double min_goal_value = 1000000000.0;
				int min_goal_index = 0;

				for(int check_goal = 0; check_goal < goal_point.size(); check_goal++)
				{

					if(10 > std::abs(goal_index - check_goal))
					{
						double rx = transform.getOrigin().x();
						double ry = transform.getOrigin().y();

						double gx = goal_point[check_goal].position[0];
						double gy = goal_point[check_goal].position[1];

						double dist = std::sqrt(std::pow(gx- rx, 2) + std::pow(gy - ry, 2));

						gm_map_pose.header.frame_id = "/map";
						gm_map_pose.header.stamp = ros::Time::now();
						gm_map_pose.pose.position.x = goal_point[check_goal].position[0];
						gm_map_pose.pose.position.y = goal_point[check_goal].position[1];
						gm_map_pose.pose.position.z = goal_point[check_goal].position[2];
						gm_map_pose.pose.orientation.x = goal_point[check_goal].orientation[0];
						gm_map_pose.pose.orientation.y = goal_point[check_goal].orientation[1];
						gm_map_pose.pose.orientation.z = goal_point[check_goal].orientation[2];
						gm_map_pose.pose.orientation.w = goal_point[check_goal].orientation[3];

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

						double diff_rad = atan2(gm_base_pose.pose.position.y, gm_base_pose.pose.position.x);

						if (diff_rad > M_PI)
						{
							diff_rad -= 2*M_PI;
						}
						else if (diff_rad < -M_PI)
						{
							diff_rad += 2*M_PI;
						}

						diff_rad = std::abs(diff_rad);

						double goal_value = 10.0*(diff_rad / M_PI) + (dist / 10.0);

						if(min_goal_value > goal_value)
						{
							min_goal_value = goal_value;
							min_goal_index = check_goal;
						}

						std::cout << "goal_num: " << check_goal << ", " << "value: " << goal_value << ", " << "dist: " <<  dist << ", " << "rad: " << diff_rad << std::endl;

					}

				}

				std::cout << "min_goal_num: " << min_goal_index << std::endl;

				gm_map_pose.header.frame_id = "/map";
				gm_map_pose.header.stamp = ros::Time::now();
				gm_map_pose.pose.position.x = goal_point[min_goal_index].position[0];
				gm_map_pose.pose.position.y = goal_point[min_goal_index].position[1];
				gm_map_pose.pose.position.z = goal_point[min_goal_index].position[2];
				gm_map_pose.pose.orientation.x = goal_point[min_goal_index].orientation[0];
				gm_map_pose.pose.orientation.y = goal_point[min_goal_index].orientation[1];
				gm_map_pose.pose.orientation.z = goal_point[min_goal_index].orientation[2];
				gm_map_pose.pose.orientation.w = goal_point[min_goal_index].orientation[3];


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

				std::cout << "person rotation start" << std::endl;

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

					rate.sleep();
					ros::spinOnce();
				}

				std::cout << "person rotation finish" << std::endl;

				goal.target_pose.header.frame_id = "/map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = goal_point[min_goal_index].position[0];
				goal.target_pose.pose.position.y = goal_point[min_goal_index].position[1];
				goal.target_pose.pose.position.z = goal_point[min_goal_index].position[2];
				goal.target_pose.pose.orientation.x = goal_point[min_goal_index].orientation[0];
				goal.target_pose.pose.orientation.y = goal_point[min_goal_index].orientation[1];
				goal.target_pose.pose.orientation.z = goal_point[min_goal_index].orientation[2];
				goal.target_pose.pose.orientation.w = goal_point[min_goal_index].orientation[3];
				ac.sendGoal(goal);

				std::cout << "person navigation just now goal" << std::endl;
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

					double wx = goal_point[min_goal_index].position[0];
					double wy = goal_point[min_goal_index].position[1];
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

				std::cout << "person navigation just now goal finish" << std::endl;

				sleep(1);

				gm_map_pose.header.frame_id = "/map";
				gm_map_pose.header.stamp = ros::Time::now();
				gm_map_pose.pose.position.x = goal_point[min_goal_index+1].position[0];
				gm_map_pose.pose.position.y = goal_point[min_goal_index+1].position[1];
				gm_map_pose.pose.position.z = goal_point[min_goal_index+1].position[2];

				std::cout << "person rotation to next goal" << std::endl;

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

					rate.sleep();
					ros::spinOnce();
				}

				std::cout << "person rotation to next goal finish" << std::endl;

				person_flag = false;

				i = min_goal_index + 1;

				break;
			}

			ros::spinOnce();
		}
	}

	return 0;
}
