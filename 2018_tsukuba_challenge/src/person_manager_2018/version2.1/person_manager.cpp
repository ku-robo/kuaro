/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/30
* <version>		v2.1
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "stdafx.hpp"
#include "person_manager.hpp"

// コンストラクタでパラメータやファイルの読み込みを行う
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
	ph.param("danger_scan_count", danger_scan_count, 50);
	ph.param("danger_range_x", danger_range_x, 0.3);
	ph.param("danger_range_y", danger_range_y, 0.4);
	std::cout << "read: " << read_file << std::endl;
	std::cout << "distance: " << distance << std::endl;
	std::cout << "time_limit: " << time_limit << std::endl;
	std::cout << "back_vel: " << back_vel << std::endl;
	std::cout << "start_point: " << start_point << std::endl;
	std::cout << "map_flag: " << map_flag << std::endl;
	std::cout << "danger_scan_count: " << danger_scan_count << std::endl;
	std::cout << "danger_range_x: " << danger_range_x << std::endl;
	std::cout << "danger_range_y: " << danger_range_y << std::endl;


	// waypointが書かれているファイルが存在するか調べる
	FILE* fp;
	fp = fopen(read_file.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("file path error!");
		return;
	}
	GetPoint tmp;
	while(fscanf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &tmp.position[0], &tmp.position[1], &tmp.position[2], &tmp.orientation[0], &tmp.orientation[1], &tmp.orientation[2], &tmp.orientation[3]) != EOF)
	{
		goal_point.push_back(tmp);
	}
	fclose(fp);


	// min_obstacleの設定値を格納する
	if(map_flag == 1)
	{
		min_obstacle_setter.push_back(std::make_pair(0, 0.8));
		min_obstacle_setter.push_back(std::make_pair(41, 0.1));
		min_obstacle_setter.push_back(std::make_pair(60, 0.8));
		min_obstacle_setter.push_back(std::make_pair(76, 0.01));
		min_obstacle_setter.push_back(std::make_pair(90, 0.5));

		// rouka
		// min_obstacle_setter.push_back(std::make_pair(0, 0.5));
		// min_obstacle_setter.push_back(std::make_pair(3, 0.5));
	}

	// flagの初期化
	person_flag = false;
	back_danger_flag = false;
	now_min_obstacle_value = -1.0;
	min_obs_str = "rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist ";

	// パブリッシャーとサブスクライバの設定
	cmd_pub = cmd_node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	person_sub = perosn_node.subscribe("/person_point", 1, &C_person_manager::get_person_callback, this);

	// scan_backを取得する
	scan_sub = scan_node.subscribe("/scan_back", 1, &C_person_manager::get_scan_back_callback, this);
	cloud_pub = scan_node.advertise<sensor_msgs::PointCloud2>("/danger_cloud", 10);
}


C_person_manager::~C_person_manager(void)
{

}

// 人物データをここで拾う
void C_person_manager::get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point)
{
	// 人物がいる区間の waypoint なら人物までナビゲーションを行う
	if(28 < goal_index && goal_index < 41)
	{
		std::cout << "Search Target!" << std::endl;
	}
	else
	{
		return;
	}

	// サブスクライバのデータを受け取る．　z軸の座標はヨー角を入れている（一応）
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

// scanのサブスクライブを行う
void C_person_manager::get_scan_back_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	sensor_msgs::PointCloud tmp_cloud1;
	sensor_msgs::PointCloud2 tmp_cloud2;

	while(ros::ok())
	{
		try
		{
			scan_listener.waitForTransform(scan_msg->header.frame_id.c_str(), scan_msg->header.frame_id.c_str(), scan_msg->header.stamp, ros::Duration(0.1));
			scan_projector.transformLaserScanToPointCloud(scan_msg->header.frame_id, *scan_msg, tmp_cloud1, scan_listener);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		break;
	}

	sensor_msgs::convertPointCloudToPointCloud2(tmp_cloud1,tmp_cloud2);
	pcl::fromROSMsg(tmp_cloud2, *scan_cloud);

	int cloud_size = (int) scan_cloud->points.size();

	int danger_count = 0;
	for(int i = 0; i < cloud_size; i++)
	{
		float point_x = scan_cloud->points[i].x;
		float point_y = scan_cloud->points[i].y;

		if(danger_range_x > point_x && danger_range_y > std::abs(point_y))
		{
			pcl::PointXYZ point;
			point.x = scan_cloud->points[i].x;
			point.y = scan_cloud->points[i].y;
			point.z = scan_cloud->points[i].z;
			pub_cloud->push_back(point);
			danger_count++;
		}
	}

	//std::cout << "danger_count: " << danger_count << std::endl;

	if(danger_count > danger_scan_count)
	{
		back_danger_flag = true;
	}
	else
	{
		back_danger_flag = false;
	}

	sensor_msgs::PointCloud2::Ptr cloud_msg (new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*pub_cloud, *cloud_msg);
	cloud_msg->header.frame_id = scan_msg->header.frame_id;
	cloud_msg->header.seq = scan_msg->header.seq;
	cloud_msg->header.stamp = scan_msg->header.stamp;
	cloud_pub.publish(*cloud_msg);

	return;
}

// tebローカルプランナーの min_obstacle_dist をここで設定する
void C_person_manager::min_obstacle_set(int goal_index)
{
	std::string tmp_str;

	int set_num = (int)(min_obstacle_setter.size());

	for(int i = 0; i < set_num-1; ++i)
	{
		if(min_obstacle_setter[i].first <= goal_index && goal_index < min_obstacle_setter[i+1].first)
		{
			if(min_obstacle_setter[i].second == now_min_obstacle_value)
			{
				return;
			}

			now_min_obstacle_value = min_obstacle_setter[i].second;
			std::string tmp_str = min_obs_str + std::to_string(min_obstacle_setter[i].second);
			system(tmp_str.c_str());
		}
	}

	if(min_obstacle_setter[set_num-1].first <= goal_index)
	{
		if(min_obstacle_setter[set_num-1].second == now_min_obstacle_value)
		{
			return;
		}

		now_min_obstacle_value = min_obstacle_setter[set_num-1].second;
		std::string tmp_str = min_obs_str + std::to_string(min_obstacle_setter[set_num-1].second);
		system(tmp_str.c_str());
	}

	return;
}

// map上の座標を取得する
tf::StampedTransform C_person_manager::get_tf(std::string input_frame, std::string output_frame, ros::Time input_time)
{
	while(ros::ok())
	{
		try
		{
			listener.lookupTransform(output_frame, input_frame, input_time, transform);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		break;
	}

	return transform;
}

// 自分で入力した地点を任意のフレームでの座標に変換する
geometry_msgs::PoseStamped C_person_manager::get_gm(geometry_msgs::PoseStamped input_pose, std::string input_frame, std::string output_frame, ros::Time input_time)
{
	while(ros::ok())
	{
		try
		{
			listener.transformPose(output_frame, input_time, input_pose, input_frame, gm_pose);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		break;
	}

	return gm_pose;
}

// cmd_velを強制的にパブリッシュする
void C_person_manager::execute_cmd(double trans_vel, double rot_vel, double limit)
{
	ros::Time start_time, current_time;
	start_time = ros::Time::now();
	double dt;
	ros::Rate cmd_rate(30);

	bool trans_flag = false;
	bool rot_flag = false;

	if(std::abs(trans_vel) > 0.0)
	{
		trans_flag = true;
		std::cout << "back now !! " << std::endl;
	}
	else if (std::abs(rot_vel) > 0.0)
	{
		rot_flag = true;
		std::cout << "rotate now !! " << std::endl;
	}

	if(trans_flag == true && back_danger_flag == true)
	{
		// 後に何かあるのでbackをやめる
		sleep(3.0);
		return;
	}

	while (ros::ok())
	{
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = trans_vel;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = rot_vel;

		if(trans_flag == true && back_danger_flag == true)
		{
			cmd_vel.linear.x = 0.0;
		}

		cmd_pub.publish(cmd_vel);
		current_time = ros::Time::now();
		dt = (current_time - start_time).toSec();
		if (dt > limit) break;

		cmd_rate.sleep();
		ros::spinOnce();
	}

	std::cout << "cmd finish !! " << std::endl;

	return;
}

// 角度を丸め込む関数
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

// 実質的にプログラムの本体部分
void C_person_manager::run()
{
	ros::Rate main_rate(30);
	goal_index = -1;

	// move_baseのクライアントを作成する
	MoveBaseClient ac("move_base", true);

	// クライアント・サービスが立ち上がるまで待機する
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	std::cout << "READY" << std::endl;

	for(int i = start_point; i < (int)(goal_point.size()); i++)
	{
		goal_index = i;

		// ゴールを送るための move_baseの変数
		move_base_msgs::MoveBaseGoal goal;

		// min_obstacle distを動的に変更している
		min_obstacle_set(goal_index);

		// テキストから読んだゴールを変数に移している
		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = goal_point[i].position[0];
		goal.target_pose.pose.position.y = goal_point[i].position[1];
		goal.target_pose.pose.position.z = goal_point[i].position[2];
		goal.target_pose.pose.orientation.x = goal_point[i].orientation[0];
		goal.target_pose.pose.orientation.y = goal_point[i].orientation[1];
		goal.target_pose.pose.orientation.z = goal_point[i].orientation[2];
		goal.target_pose.pose.orientation.w = goal_point[i].orientation[3];

		std::cout << i << ": " << goal_point[i].position[0] << ", " << goal_point[i].position[1] << std::endl;

		ac.sendGoal(goal);

		while(ros::ok())
		{
			actionlib::SimpleClientGoalState state = ac.getState();

			// 人物を見つけていない場合の通常処理
			if(person_flag == false)
			{
				// 現在の map上の座標を取得
				tf::StampedTransform now_tf = get_tf("base_link", "map", ros::Time(0));

				// 現在位置からゴールまでの距離を計算する
				double dist = hypot(goal_point[i].position[0] - now_tf.getOrigin().x(), goal_point[i].position[1] - now_tf.getOrigin().y());

				if(dist < distance)
				{
					// ゴールに辿り着いたら，キャンセル処理をして，forループを進める
					ac.cancelAllGoals();
					break;
				}

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && i != (goal_point.size() - 1))
				{
					// いらない処理かも？
					std::cout << "????" << std::endl;
					ac.sendGoal(goal);
				}
				else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					// ABORTEDになったら，ゴールをキャンセルし，0.5m後退して，再度ゴールを送りなおす
					ROS_ERROR("ABOoooooooRTED");
					ac.cancelAllGoals();
					now_min_obstacle_value = 0.1;
					system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.1");
					execute_cmd(back_vel, 0.0, time_limit);
					ac.sendGoal(goal);
				}
				else if(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE && ac.getState() != actionlib::SimpleClientGoalState::PENDING)
				{
					// ゴールがアクティブでなく，PENDING（ゴール送り中）でなければ，再度ゴールを送りなおす
					std::cout << "No ACTIVE send goal again" << std::endl;
					ac.sendGoal(goal);
				}

			}
			else
			{
				// ここから人物までナビゲーションを行う処理が始まる
				std::cout << "detect person and navigation" << std::endl;

				// 現在向かっているすべてのゴールをキャンセルする
				ac.cancelAllGoals();
				sleep(1);

				// 探索対象へ向かうゴールを用意する
				move_base_msgs::MoveBaseGoal person_goal;

				// tfによる座標変換の実装
				geometry_msgs::PoseStamped gm_input_pose;
				geometry_msgs::PoseStamped gm_output_pose;

				std::cout << "distance to target person: " << now_person.position[0] << ", " << now_person.position[1] << std::endl;

				// base_link座標系 から見た人物の座標を map座標系に変換する
				gm_input_pose.header.frame_id = "/base_link";
				gm_input_pose.header.stamp = person_time;
				gm_input_pose.pose.position.x = now_person.position[0];
				gm_input_pose.pose.position.y = now_person.position[1];
				gm_input_pose.pose.position.z = 0.0;
				gm_input_pose.pose.orientation.x = 0.0;
				gm_input_pose.pose.orientation.y = 0.0;
				gm_input_pose.pose.orientation.z = 0.0;
				gm_input_pose.pose.orientation.w = 1.0;

				// tfの実行
				gm_output_pose = get_gm(gm_input_pose, "/base_link", "/map", person_time);

				// 現在の map上の座標を取得
				tf::StampedTransform now_tf = get_tf("base_link", "map", person_time);

				// 人物座標から現在地までの角度を求める
				double map_dx = gm_output_pose.pose.position.x - now_tf.getOrigin().x();
				double map_dy = gm_output_pose.pose.position.y - now_tf.getOrigin().y();
				geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, atan2(map_dy, map_dx));

				// 人物までのゴールを設定する
				person_goal.target_pose.header.frame_id = "/map";
				person_goal.target_pose.header.stamp = ros::Time::now();
				person_goal.target_pose.pose.position.x = gm_output_pose.pose.position.x;
				person_goal.target_pose.pose.position.y = gm_output_pose.pose.position.y;
				person_goal.target_pose.pose.position.z = 0.0;
				person_goal.target_pose.pose.orientation.x = q.x;
				person_goal.target_pose.pose.orientation.y = q.y;
				person_goal.target_pose.pose.orientation.z = q.z;
				person_goal.target_pose.pose.orientation.w = q.w;

				// 人物までナビゲーションするときは min_obstacle_dist を 0.3 に下げる
				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.3");

				ac.sendGoal(person_goal);

				// 人物までナビゲーションを行う
				ros::Time person_start_time, person_current_time;
				person_start_time = ros::Time::now();
				while(1)
				{
					// 現在の map上の座標を取得
					now_tf = get_tf("base_link", "map", ros::Time(0));

					// 現在位置からゴールまでの距離を計算する
					double dist = hypot(now_person.position[0] - now_tf.getOrigin().x(), now_person.position[1] - now_tf.getOrigin().y());

					if(dist < 0.9)
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
					else if(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE && ac.getState() != actionlib::SimpleClientGoalState::PENDING)
					{
						// ゴールがアクティブでなく，PENDING（ゴール送り中）でなければ，再度ゴールを送りなおす
						std::cout << "No ACTIVE send goal again" << std::endl;
						ac.sendGoal(person_goal);
					}

					person_current_time = ros::Time::now();
					double dt = (person_current_time - person_start_time).toSec();
					if (dt > 180.0)
					{
						std::cout << "Time Over !!" << std::endl;
						ac.cancelAllGoals();
						break;
					}

					main_rate.sleep();
					ros::spinOnce();
				}

				sleep(1);

				// 人物までのナビゲーションが終わったら，現在位置から少し下がる．
				execute_cmd(back_vel, 0.0, time_limit*1.5);

				// 現在の map上の座標を取得
				now_tf = get_tf("base_link", "map", ros::Time(0));

				// 次に向かうウェイポイントを決める 佐藤作
				int initial_i = i;
				int waypoint_num_limit = 5;
				double th_ono = 0.0;
				double robo_x = now_tf.getOrigin().x();
				double robo_y = now_tf.getOrigin().y();
				double min_waypoint_dist = hypot(goal_point[i].position[0] - robo_x, goal_point[i].position[1] - robo_y); // 最も近いウェイポイントの距離（初期値は人物探索前に行こうとしていたウェイポイント）
				double th_person = atan2(person_goal.target_pose.pose.position.y - robo_y, person_goal.target_pose.pose.position.x - robo_x); // ロボットの位置と人物の位置の角度
				for(int j = i; j < i + waypoint_num_limit ; j++)
				{
					double waypoint_dist = hypot(goal_point[j].position[0] - robo_x, goal_point[j].position[1] - robo_y);	// ウェイポイントの距離
					double th_waypoint = atan2(goal_point[j].position[1] - robo_y, goal_point[j].position[0] - robo_x);		// ロボットの位置とウェイポイントの位置の角度
					double th_person_to_waypoint = rounding(th_waypoint - th_person); 																		// ロボットの位置と人物の位置の角度を基準とした、ロボットの位置とウェイポイントの位置の角度

					if (waypoint_dist <= min_waypoint_dist && std::abs(th_person_to_waypoint) >= M_PI/4 || j == initial_i)
					{
						min_waypoint_dist = waypoint_dist;
						th_ono = th_person_to_waypoint;
						i = j;
					}
				}

				std::cout << "init goal num " << initial_i <<  std::endl;
				std::cout << "decide goal num " << i << std::endl;

				//現在位置から向かっていたゴールまでの角度を計算するために，base_link座標系でゴールの座標を取得する．
				gm_input_pose.header.frame_id = "/map";
				gm_input_pose.header.stamp = ros::Time::now();
				gm_input_pose.pose.position.x = goal_point[i].position[0];
				gm_input_pose.pose.position.y = goal_point[i].position[1];
				gm_input_pose.pose.position.z = goal_point[i].position[2];
				gm_input_pose.pose.orientation.x = goal_point[i].orientation[0];
				gm_input_pose.pose.orientation.y = goal_point[i].orientation[1];
				gm_input_pose.pose.orientation.z = goal_point[i].orientation[2];
				gm_input_pose.pose.orientation.w = goal_point[i].orientation[3];

				// tfの実行
				gm_output_pose = get_gm(gm_input_pose, "/map", "/base_link", ros::Time(0));

				//ゴールまでロボットを向かせる
				double forward_goal_rad = atan2(gm_output_pose.pose.position.y, gm_output_pose.pose.position.x);
				double forward_goal_dist = hypot(gm_output_pose.pose.position.y, gm_output_pose.pose.position.x);
				double rot_need_time = std::abs(forward_goal_rad / 0.2);

				if(forward_goal_dist < distance)
				{
					// 次のゴールまで近ければ何もしない
				}
				else
				{
					if(forward_goal_rad > 0.0)
					{
						execute_cmd(0.0, 0.2, rot_need_time);
					}
					else
					{
						execute_cmd(0.0, -0.2, rot_need_time);
					}
				}

				//ゴールを与え直す
				goal.target_pose.header.frame_id = "/map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = goal_point[i].position[0];
				goal.target_pose.pose.position.y = goal_point[i].position[1];
				goal.target_pose.pose.position.z = goal_point[i].position[2];
				goal.target_pose.pose.orientation.x = goal_point[i].orientation[0];
				goal.target_pose.pose.orientation.y = goal_point[i].orientation[1];
				goal.target_pose.pose.orientation.z = goal_point[i].orientation[2];
				goal.target_pose.pose.orientation.w = goal_point[i].orientation[3];

				ac.sendGoal(goal);

				// ゴールまでナビゲーションを行う
				while(ros::ok())
				{
					state = ac.getState();

					// 現在の map上の座標を取得
					tf::StampedTransform now_tf = get_tf("base_link", "map", ros::Time(0));

					// 現在位置からゴールまでの距離を計算する
					double dist = hypot(goal_point[i].position[0] - now_tf.getOrigin().x(), goal_point[i].position[1] - now_tf.getOrigin().y());

					if(dist < distance)
					{
						// ゴールに辿り着いたら，キャンセル処理をして，forループを進める
						ac.cancelAllGoals();
						break;
					}

					if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && i != (goal_point.size() - 1))
					{
						// いらない処理かも？
						std::cout << "????" << std::endl;
						ac.sendGoal(goal);
					}
					else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
					{
						// ABORTEDになったら，ゴールをキャンセルし，0.5m後退して，再度ゴールを送りなおす
						ROS_ERROR("ABOoooooooRTED");
						ac.cancelAllGoals();
						execute_cmd(back_vel, 0.0, time_limit);
						ac.sendGoal(goal);
					}
					else if(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE && ac.getState() != actionlib::SimpleClientGoalState::PENDING)
					{
						// ゴールがアクティブでなく，PENDING（ゴール送り中）でなければ，再度ゴールを送りなおす
						std::cout << "No ACTIVE send goal again" << std::endl;
						ac.sendGoal(goal);
					}

					main_rate.sleep();
					ros::spinOnce();
				}

				sleep(1);

				// 次のゴールまでの角度を計算するために，base_link座標系でゴールの座標を取得する．
				gm_input_pose.header.frame_id = "/map";
				gm_input_pose.header.stamp = ros::Time::now();
				gm_input_pose.pose.position.x = goal_point[i+1].position[0];
				gm_input_pose.pose.position.y = goal_point[i+1].position[1];
				gm_input_pose.pose.position.z = goal_point[i+1].position[2];
				gm_input_pose.pose.orientation.x = goal_point[i+1].orientation[0];
				gm_input_pose.pose.orientation.y = goal_point[i+1].orientation[1];
				gm_input_pose.pose.orientation.z = goal_point[i+1].orientation[2];
				gm_input_pose.pose.orientation.w = goal_point[i+1].orientation[3];

				// tfの実行
				gm_output_pose = get_gm(gm_input_pose, "/map", "/base_link", ros::Time(0));

				// 次のゴールの方をロボットに向かせる
				forward_goal_rad = atan2(gm_output_pose.pose.position.y, gm_output_pose.pose.position.x);
				forward_goal_dist = hypot(gm_output_pose.pose.position.y, gm_output_pose.pose.position.x);
				rot_need_time = std::abs(forward_goal_rad / 0.2);

				// 次のゴールの方を向かせる
				if(forward_goal_rad > 0.0)
				{
					execute_cmd(0.0, 0.2, rot_need_time);
				}
				else
				{
					execute_cmd(0.0, -0.2, rot_need_time);
				}

				system("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist 0.8");

				person_flag = false;
				break;
			}

			main_rate.sleep();
			ros::spinOnce();

			// 最初のwhileループの終わり
		}

		// forループの終わり
	}

	return;
}
