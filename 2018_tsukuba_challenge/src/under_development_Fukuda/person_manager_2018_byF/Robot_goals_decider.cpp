#include <2018_tsukuba_challenge/Robot_goals_decider.hpp>

// コンストラクタ.....using_Client_(Client_name, true)これは，アクションクライアントの初期化
Robot_goals_decider::Robot_goals_decider():
person_flag_(false),	back_danger_flag_(false), goal_index_(-1),
now_try_count_(0), now_min_obstacle_value_(-1.0), ab_c_(), goal_states(FINISH),nh("~")
{
  nh.param("Client_name", Rgd_Param_const_.Client_name, std::string("move_base"));
  nh.param("read_file", Rgd_Param_const_.read_file, std::string("/home/robo/bagfile/sadahira_2/way_points_output.txt"));
  nh.param("distance", Rgd_Param_const_.distance, 2.0);
  nh.param("time_limit", Rgd_Param_const_.time_limit, 5.0);
  nh.param("back_vel", Rgd_Param_const_.back_vel, -0.1);
  nh.param("start_point", Rgd_Param_const_.start_point, 0);
  nh.param("map_flag", Rgd_Param_const_.map_flag, -1);
  nh.param("danger_scan_count", Rgd_Param_const_.danger_scan_count, 50);
  nh.param("danger_range_x", Rgd_Param_const_.danger_range_x, 0.3);
  nh.param("danger_range_y", Rgd_Param_const_.danger_range_y, 0.4);
  now_try_count_ = Rgd_Param_const_.start_point;
  Rgd_Param_const_.print_member();
  csv_reader(waypoint_box_, Rgd_Param_const_.read_file);//waypointデータをcsvから読み込む．２次元配列のVecに入れる

}

// デストラクタ
Robot_goals_decider::~Robot_goals_decider()
{

}



//処理の中心となる関数
bool Robot_goals_decider::main_mover()
{
  ros::Rate main_rate(main_rate);

  using_Client_.reset(new MoveBaseClient(Rgd_Param_const_.Client_name, true));
  // クライアント・サービスが立ち上がるまで待機する
	while(!using_Client_->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

  std::cout << "READY" << std::endl;

  while(now_try_count_ < waypoint_box_.size()){
    Get_point &now_try_point = waypoint_box_[now_try_count_];
    waypoints_inputer(main_goal_, std::string("/map"), now_try_point);
    std::cout << " waypoint_number is " << now_try_count_ << ": " << now_try_point.data[POSI_X] << ", " << now_try_point.data[POSI_Y] << std::endl;
    using_Client_->sendGoal(main_goal_);//goalを伝える

    bool Out_flag = false;
    while(ros::ok()){
      if(person_flag_ == false){
        // 人物を見つけていない時の処理
        Out_flag = MainGoal_mode(using_Client_, main_goal_, now_try_point, (now_try_count_ != (waypoint_box_.size() - 1)));
      } else {
        // 人物を見つけた時の処理
        Out_flag = PersonGoal_mode();
      }
      if(Out_flag) break;
      main_rate.sleep();
      ros::spinOnce();
    }

    now_try_count_++;//ある条件をつけて，加算するようにプログラムを組む....あくまでwaypoint_box_は，csvfile内のデータのみを取得し，他のwaypointが途中参加するように作られてないので
  }
  return true;
}


//人を検出しなかった場合の動き
bool Robot_goals_decider::MainGoal_mode(const std::unique_ptr<MoveBaseClient>& ac, const move_base_msgs::MoveBaseGoal& goal, const Get_point& now_point, bool current_state)
{
  // 現在の map上の座標を取得
  tf::StampedTransform now_tf = get_tf("map", "base_link", ros::Time(0));

  // 現在位置からゴールまでの距離を計算する
  double dist = hypot(now_point.data[POSI_X] - now_tf.getOrigin().x(), now_point.data[POSI_Y] - now_tf.getOrigin().y());

  if(dist < Rgd_Param_const_.distance)
  {
    // ゴールに辿り着いたら，キャンセル処理をして，forループを進める
    ac->cancelAllGoals();
    goal_states = FINISH;
    ab_c_.reset(goal_states);
    return true;
  }

  // if(5 < abort_count)//abort時の処理
  // {
  //   // ゴールに辿り着いたら，キャンセル処理をして，forループを進める
  //   using_Client_.cancelAllGoals();
  //   return true;
  // }

  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && current_state)
  {
    // いらない処理かも？
    std::cout << "????" << std::endl;
    ac->sendGoal(goal);

  }
  else if(ac->getState() == actionlib::SimpleClientGoalState::ABORTED)//Recoverの処理。。。。。修正すべき
  {
    // ABORTEDになったら，ゴールをキャンセルし，0.5m後退して，再度ゴールを送りなおす
    ROS_ERROR("ABOoooooooRTED");
    ac->cancelAllGoals();
    ab_c_.mover();

    // abort_count++;

    //min_obstacle_distを変更する処理を入れるかも

    //cmd_velに直接司令をおくる関数を入れる
    ac->sendGoal(goal);
  }
  else if(ac->getState() != actionlib::SimpleClientGoalState::ACTIVE && ac->getState() != actionlib::SimpleClientGoalState::PENDING)
  {
    // ゴールがアクティブでなく，PENDING（ゴール送り中）でなければ，再度ゴールを送りなおす
    std::cout << "No ACTIVE send goal again" << std::endl;
    ac->sendGoal(goal);
  }
  goal_states = NOW_MOVE;
  ab_c_.reset(goal_states);
  return false;
}


//人を検出した際の動き
bool Robot_goals_decider::PersonGoal_mode()
{

}

// map上の座標を取得する
tf::StampedTransform Robot_goals_decider::get_tf(const std::string& output_frame, const std::string& input_frame, const ros::Time &input_time)
{
	ros::Rate tf_rate(30);
  tf::StampedTransform transform;

	while(ros::ok())
	{
		try
		{
			listener_.lookupTransform(output_frame, input_frame, input_time, transform);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();

			tf_rate.sleep();
			ros::spinOnce();

			continue;
		}
		break;
	}

	return transform;
}

// 自分で入力した地点を任意のフレームでの座標に変換する
geometry_msgs::PoseStamped Robot_goals_decider::get_gm(const std::string& output_frame, const ros::Time &input_time, const geometry_msgs::PoseStamped& input_pose, const std::string& input_frame)
{
	ros::Rate tf_rate(30);
  geometry_msgs::PoseStamped gm_pose;
	while(ros::ok())
	{
		try
		{
			listener_.transformPose(output_frame, input_time, input_pose, input_frame, gm_pose);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();

			tf_rate.sleep();
			ros::spinOnce();

			continue;
		}
		break;
	}

	return gm_pose;
}

// 人物データを取得するCallback関数
void Robot_goals_decider::get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point)
{
  bool person_diff_flag = false;
  Get_point now_person;
	if(person_point_.size() > 0)
	{
    double person_diff;
		for(int i = 0; i < person_point_.size(); ++i)
		{
			person_diff = hypot(person_point_[i].data[POSI_X] - sub_point->point.x, person_point_[i].data[POSI_Y] - sub_point->point.y);
			if(10.0 > person_diff) person_diff_flag = true;
		}
	}

	if(person_diff_flag == true)
	{
		std::cout << "It is a detected person" << std::endl;
		return;
	}

	now_person.data[POSI_X] = sub_point->point.x;
	now_person.data[POSI_Y] = sub_point->point.y;
	now_person.data[POSI_Z] = sub_point->point.z;
	now_person.data[ORIEN_X] = 0.0;
	now_person.data[ORIEN_Y] = 0.0;
	now_person.data[ORIEN_Z] = 0.0;
	now_person.data[ORIEN_W] = 1.0;
	person_time_ = sub_point->header.stamp;
	person_point_.push_back(now_person);

	person_flag_ = true; // fake_obstacle need comment out
}


//csvを読み込んで落としこむ
bool Robot_goals_decider::csv_reader(std::vector< Get_point >& data_box, const std::string& path)
{
  std::ifstream ifs( path );
  if(!ifs){
    std::cout << "---ERROR---  : I can't open the file....." << std::endl;
    return false;
  }
  std::string line, str_num;
  Get_point GP_box;
  double val_d;
  while(std::getline(ifs, line)){
    std::istringstream ifs2(line);
    for(int i = 0; i < data_num; i++){
      if(std::getline(ifs2, str_num, ',')){
        val_d = std::stod(str_num);
      } else {
        val_d = 0.0;
      }
      GP_box.data[i] = val_d;
      std::cout << GP_box.data[i];
      if(i < data_num - 1) std::cout << ", ";
    }
    data_box.push_back(GP_box);
    std::cout << "\n" ;
  }
  return true;
}


//goal用......値を代入するためだけの関数
void Robot_goals_decider::waypoints_inputer(move_base_msgs::MoveBaseGoal& goal, const std::string& frame_id, const Get_point& using_data)
{
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = using_data.data[POSI_X];
  goal.target_pose.pose.position.y = using_data.data[POSI_Y];
  goal.target_pose.pose.position.z = using_data.data[POSI_Z];
  goal.target_pose.pose.orientation.x = using_data.data[ORIEN_X];
  goal.target_pose.pose.orientation.y = using_data.data[ORIEN_Y];
  goal.target_pose.pose.orientation.z = using_data.data[ORIEN_Z];
  goal.target_pose.pose.orientation.w = using_data.data[ORIEN_W];
}
//person用......値を代入するためだけの関数
void Robot_goals_decider::waypoints_inputer(move_base_msgs::MoveBaseGoal& goal, const std::string& frame_id, const Get_point& using_data, const geometry_msgs::Quaternion& Q_point)
{
  goal.target_pose.header.frame_id = frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = using_data.data[POSI_X];
  goal.target_pose.pose.position.y = using_data.data[POSI_Y];
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = Q_point.x;
  goal.target_pose.pose.orientation.y = Q_point.y;
  goal.target_pose.pose.orientation.z = Q_point.z;
  goal.target_pose.pose.orientation.w = Q_point.w;
}
//goal用......値を代入するためだけの関数
void Robot_goals_decider::waypoints_inputer(geometry_msgs::PoseStamped& gm_input_pose, const std::string& frame_id, const Get_point& using_data)
{
  gm_input_pose.header.frame_id = frame_id;
  gm_input_pose.header.stamp = ros::Time::now();
  gm_input_pose.pose.position.x = using_data.data[POSI_X];
  gm_input_pose.pose.position.y = using_data.data[POSI_Y];
  gm_input_pose.pose.position.z = using_data.data[POSI_Z];
  gm_input_pose.pose.orientation.x = using_data.data[ORIEN_X];
  gm_input_pose.pose.orientation.y = using_data.data[ORIEN_Y];
  gm_input_pose.pose.orientation.z = using_data.data[ORIEN_Z];
  gm_input_pose.pose.orientation.w = using_data.data[ORIEN_W];
}
//person用......値を代入するためだけの関数
void Robot_goals_decider::waypoints_inputer(geometry_msgs::PoseStamped& gm_input_pose, const ros::Time& person_time, const std::string& frame_id, const Get_point& using_data)
{
  gm_input_pose.header.frame_id = frame_id;
  gm_input_pose.header.stamp = person_time;
  gm_input_pose.pose.position.x = using_data.data[POSI_X];
  gm_input_pose.pose.position.y = using_data.data[POSI_Y];
  gm_input_pose.pose.position.z = 0.0;
  gm_input_pose.pose.orientation.x = 0.0;
  gm_input_pose.pose.orientation.y = 0.0;
  gm_input_pose.pose.orientation.z = 0.0;
  gm_input_pose.pose.orientation.w = 1.0;
}
