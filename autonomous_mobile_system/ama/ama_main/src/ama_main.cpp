#include <iostream>
#include <fstream>
#include <ama_main/ama_main.hpp>

namespace ama_main{
  AMAMain::AMAMain(tf::TransformListener& tf):
  tf_(tf),  default_str_("default"),end_func_str_("FIN"),
  ama_funcs_loader_("ama_core", "ama_core::AMAFunc"),
  ama_moves_loader_("ama_core", "ama_core::AMAMove"),
  wp_nextfunc_flag_(false), advance_count_(0),
  waypoint_flag_(false), movegoal_flag_(false), func_flag_(false),
  func_next_name_(default_str_), break_flag_(false), aborted_flag_(false),exinstruct_flag_(false)
  {
    // launchファイルで設定するパラメータ...まだまって20190915
	  ros::NodeHandle private_nh("~");
    std::string nextfunc_map_file="";
    float waypoint_angle_threshold;
    private_nh.param("nextfunc_map_file_name", nextfunc_map_file, std::string("/home/robo/ama_test/nextfunc_map_filet.txt"));// file name
    private_nh.param("distance_between_target_positions", dist_target_posi_, 3.5f);// [m]
    private_nh.param("target_change_distance", target_change_dist_, 0.8f);// [m]
    private_nh.param("waypoint_angle_threshold", waypoint_angle_threshold, 12.5f);//[degree]
    private_nh.param("wp_goal_judg_threshold_ratio", wp_goal_judg_threshold_ratio_, 0.75f);//[-]
    wp_angle_threshold_ = waypoint_angle_threshold/180*M_PI;//[rad]

    // WayPoint marker publisher
    marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("waypoint_marker", 1);
    // Set up Points
		points_.header.frame_id = "/map";
		points_.header.stamp = ros::Time::now();
		points_.ns = "POINTS";
		points_.color.b = 1.0;
		points_.color.a = 1.0;
		points_.scale.x = 0.3;
		points_.scale.y = 0.3;
		points_.scale.z = 0.3;
		points_.action = visualization_msgs::Marker::ADD;
		points_.type = visualization_msgs::Marker::POINTS;
		points_.pose.orientation.w = 1.0;

    // prepare wp_nextfunc_map_....
    std::ifstream ifs(nextfunc_map_file);
    std::string line, str_num;
    while(std::getline(ifs, line)){
      int count = 0;
      unsigned int wp_index = 0;
      std::string wp_next_func;
      std::istringstream ifs2(line);
      while(std::getline(ifs2, str_num, ',')){
        count++;
        if( count == 1){
          wp_index = std::stoi(str_num);
        }else if( count == 2){
          wp_next_func = str_num;
          break;
        }
      }
      wp_nextfunc_vec_.push_back(std::make_pair(wp_index, wp_next_func));
      wp_nextfunc_flag_ =true;
    }
    if( wp_nextfunc_vec_.size() == 0 ){
      wp_nextfunc_vec_.push_back(std::make_pair(-1, "default"));
    }
    ROS_INFO(" [AMAMain][Func:initialize] SUCCEEDED LOADING WAYPOINT FILE ");
    if(!loadAMAFuncs(private_nh, points_)){
      loadDefaultFunc(points_);
    }
    if(!loadAMAMoves(private_nh)){
      loadDefaultMove();
    }

  }

  AMAMain::~AMAMain()
  {

    waypoint_thread_->interrupt();
    waypoint_thread_->join();

    delete waypoint_thread_;

    ama_funcs_map_.clear();
    ama_moves_map_.clear();
  }

  void AMAMain::run(void)
  {
    // set up thread
    waypoint_thread_ = new boost::thread(boost::bind(&AMAMain::WayPointThread, this));
    bool func_fin_flag = true;// check if the mode is finished.
    std::string current_func_name = default_str_;
    std::string current_move_name = default_str_;

    ROS_INFO(" [AMAMain][Func:run] START RUNNING 'run'");
    while(ros::ok()){

      // PHASE DETERMINE WHICH FUNCTION TO USE
      ROS_INFO(" [AMAMain][Func:run] PHASE TO DETERMINE WHICH FUNCTION TO USE");
      while(ros::ok()){

        // CHECK NEXT FUNCTION PHASE
        ROS_INFO(" [AMAMain][Func:run] CHECK NEXT FUNCTION PHASE ");
        // check if the function is changed in the scope
        {
          boost::recursive_mutex::scoped_lock func_l(func_mutex_);
          if( func_flag_ ){
            func_flag_ = false;
            current_func_name = func_next_name_;
          }
        }

        // Break if the current mode is END
        if( current_func_name == end_func_str_ ){
          ROS_INFO(" [AMAMain][Func:run] TERMINATE ALL PROCESSING '%s'", current_func_name.c_str());
          return;
        }

        // RUN FUNCTION PHASE
        ROS_INFO(" [AMAMain][Func:run] FUNCTION PHASE use '%s'", current_func_name.c_str());
        // if func_fin_flag is true, the current function mode is continued.
        func_fin_flag = ama_funcs_map_[current_func_name]->func_work();
        // check update or maintain
        if( ama_funcs_map_[current_func_name]->update_flag() ){
          boost::recursive_mutex::scoped_lock lock(waypoint_mutex_);
          waypoint_flag_ = true;
          waypoint_map_[current_func_name] = ama_funcs_map_[current_func_name]->GetPoints();
        }else if( ama_funcs_map_[current_func_name]->maintain_flag() ){
          boost::recursive_mutex::scoped_lock lock(waypoint_mutex_);
          waypoint_flag_ = true;
        }
        // get next moving method name
        current_move_name = ama_funcs_map_[current_func_name]->get_move_method();

        // if the current mode has finished, set change_func_flag_ to true.
        // If change_func_flag_ is true, it means suggesting the default mode.
        if(func_fin_flag){
          ROS_INFO(" [AMAMain][Func:run] Function '%s' finished", current_func_name.c_str());
          boost::recursive_mutex::scoped_lock func_l(func_mutex_);
          func_next_name_ = default_str_;
          func_flag_ = true;
        }else{
          ROS_INFO(" [AMAMain][Func:run] Function '%s' continued", current_func_name.c_str());
          break;
        }

      }


      // check if the current move name exists.
      if( ama_moves_map_.count(current_move_name) == 0){
        ROS_INFO(" [AMAMain][Func:run] There is no move-plugin '%s'. Use the default move-plugin instead.", current_move_name.c_str());
        current_move_name = default_str_;
      }

      // RUN FUNCTION PHASE
      ROS_INFO(" [AMAMain][Func:run] MOVE PHASE use '%s'", current_move_name.c_str());
      ama_moves_map_[current_move_name]->move_work(movegoal_mutex_, break_mutex_, aborted_mutex_, exinstruct_mutex_);
      ROS_INFO(" [AMAMain][Func:run] MOve PHASE PROCESSING");
    }
  }

  void AMAMain::WayPointThread(void)
  {
    // param
    ros::NodeHandle n;
    ros::Rate wpget_loop(5);
    ros::Rate wpmg_loop(10);
    std::string uf_name = "";
    ama_struct::GetPoint current_goal;
    unsigned int wp_count = 0;
    bool last_flag = false;
    float target_change_dist = target_change_dist_;

    // prepare a map to hold the index for each mode
    std::map<std::string, unsigned int> index_map;
    auto begin = waypoint_map_.begin();
    auto end = waypoint_map_.end();
    for(auto itr = begin; itr!=end; itr++){
      index_map[itr->first] = 0;
    }

    ROS_INFO(" [AMAMain][SubThread:WayPointThread] START RUNNING 'WayPointThread'");
    while(n.ok()){
      // PHASE SELECTING WAYPOINT VECTOR
      ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE SELECTING WAYPOINT VECTOR");
      // check wether waypoint_flag_ is true. if true, we can use waypoint_map_.
      boost::unique_lock<boost::recursive_mutex> waypoint_l(waypoint_mutex_);
      while(ros::ok()){
        if(waypoint_flag_){
          waypoint_flag_ = false;
          waypoint_l.unlock();
          break;
        }
        waypoint_l.unlock();
        wpget_loop.sleep();
        waypoint_l.lock();
      }
      // set func_next_name_ to uf_name
      {
        boost::recursive_mutex::scoped_lock lock(func_mutex_);
        uf_name = func_next_name_;
      }
      // get vector pointer and set value to using_func_vec.
      waypoint_l.lock();
      const std::vector<ama_struct::GetPoint>& using_func_vec = waypoint_map_[uf_name];
      waypoint_l.unlock();
      // set the current func index to start_index
      unsigned int& start_index = index_map[uf_name];
      const int& wp_nf_index = wp_nextfunc_vec_[wp_count].first;
      const std::string& wp_nf_name = wp_nextfunc_vec_[wp_count].second;

      // PHASE USING WAYPOINT VECTOR
      ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE GETTING NEXT WAYPOINT...First");
      // give First waypoint
      // get current "/base_link" position on the "/map"
      current_pose_ = get_origin_position_now();
      // get waypoint. false means an error has occured
      proccesing2get_waypoint(current_goal, target_change_dist, using_func_vec, start_index, last_flag, uf_name);

      while(ros::ok()){
        // if wp_goal_judg_threshold_flag_ is true, Make a goal determination at a certain distance from the goal
        if( current_goal.differ(current_pose_).xy_hypot() < target_change_dist ){
          // PHASE GETTING NEXT WAYPOINT
          ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE REACHING CURRENT WAYPOINT");
          // move from current waypoint to next waypoint
          start_index = start_index + advance_count_;
          if( last_flag ){
            last_flag = false;
            boost::unique_lock<boost::recursive_mutex> break_l(break_mutex_);
            break_flag_ = true;
            break_l.unlock();
          }
          if(wp_nextfunc_flag_){
            // check wether to move from the current func to the next func
            if( wp_nf_index == start_index ){
              boost::unique_lock<boost::recursive_mutex> func_l(func_mutex_);
              func_flag_ = true;
              func_next_name_ = wp_nf_name;
              func_l.unlock();
              boost::unique_lock<boost::recursive_mutex> break_l(break_mutex_);
              break_flag_ = true;
              break_l.unlock();
              // PHASE MOVING CURRENT FUNC TO NEXT FUNC
              ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE MOVING CURRENT FUNC TO NEXT FUNC");
              // for next time. move forward waypoint
              start_index = start_index+1;
              wp_count++;
              break;
            }
          }
          // PHASE GETTING NEXT WAYPOINT
          ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE GETTING NEXT WAYPOINT");
          // get next waypoint
          proccesing2get_waypoint(current_goal, target_change_dist, using_func_vec, start_index, last_flag, uf_name);
        }
        // wp_goal_judg_threshold_flag_ がfalseの場合は,必ずそのwaypointについたらtrueにもどす
        boost::unique_lock<boost::recursive_mutex> aborted_l(aborted_mutex_);
        if(aborted_flag_){
          aborted_flag_ = false;
          aborted_l.unlock();
          // PHASE DEALING WITH ABORTED
          ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE DEALING WITH ABORTED");
          // get next goal.....in the future, I want to improve this processing.
          // for example, use waypoints that consider LRF information
          // get next waypoint
          proccesing2get_waypoint(current_goal, target_change_dist, using_func_vec, start_index, last_flag,  uf_name);
        }else{
          aborted_l.unlock();
        }
        wpmg_loop.sleep();
      }
    }
  }

  void AMAMain::proccesing2get_waypoint(ama_struct::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag, const std::string& using_func_name )
  {
    if( !get_waypoint(get_wp, target_change_dist, using_func_vec, current_index, last_flag) ){
      // This error occurs when there is an invalid current_index
      ROS_WARN(" [AMAMain][SubThread:WayPointThread][Func:proccesing2get_waypoint] Index Error... Func '%s' MaxIndex '%d', StartIndex '%d", using_func_name.c_str(), (int)using_func_vec.size(), current_index);
      boost::unique_lock<boost::recursive_mutex> func_l(func_mutex_);
      func_flag_ = true;
      func_next_name_ = end_func_str_;
      func_l.unlock();
      boost::recursive_mutex::scoped_lock break_l(break_mutex_);
      break_flag_ = true;
    }
    // tell new goal information to AMAMove
    {
      boost::recursive_mutex::scoped_lock mg_l(movegoal_mutex_);
      movegoal_flag_ = true;
      movegoal_next_g_ = get_wp;
    }
    visualization_msgs::Marker& current_points = eachfunc_points_[using_func_name];
    geometry_msgs::Point gm_point;
  	gm_point.x = get_wp.position_x;
  	gm_point.y = get_wp.position_y;
  	gm_point.z = 0.25;
  	current_points.points.push_back(gm_point);
    marker_pub_.publish(current_points);
  }

  bool AMAMain::get_waypoint(ama_struct::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag)
  {
    // calculate difference between current pose and current way point
    get_wp = using_func_vec[current_index].differ(current_pose_);
    float current_dist_target_posi = get_wp.xy_hypot();
    advance_count_ = 1;
    target_change_dist = target_change_dist_;
    // compare with dist_target_posi_
    if( dist_target_posi_ < current_dist_target_posi ){
      // In range...pattern.1
      ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 1");
      get_wp.xy_flexibility(dist_target_posi_);
      get_wp += current_pose_;
      advance_count_ = 0;
      return true;
    }else if( current_dist_target_posi < dist_target_posi_ ){
      unsigned int max_size = using_func_vec.size() - 1;
      if( current_index < max_size ){
        // Out of range...have 3 patterns

        if( using_func_vec[current_index+1].differ(current_pose_).xy_hypot() < dist_target_posi_ ){
          // longer than the distance between 2 waypoints ...pattern.4
          ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 4");
          get_wp = using_func_vec[current_index];
          return true;
        }
        ama_struct::GetPoint get_wp2 = using_func_vec[current_index+1].differ(get_wp);
        float open_angle = std::abs(get_wp.yaw - get_wp2.yaw);
        if( open_angle < wp_angle_threshold_){
          // Almost straight. so create new waypoint.
          ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 3");
          get_wp2.xy_flexibility(dist_target_posi_);
          get_wp = using_func_vec[current_index];
          get_wp += get_wp2;
          get_wp.set_orientation_Yonly(get_wp2.yaw);
          return true;
        }else{
          // it's bent too much. so use current index(start_index) waypoint
          ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 2");
          get_wp = using_func_vec[current_index];
          return true;
        }
      }else if(current_index == max_size){
        // this index is last index
        // equivalent...pattern.5
        ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 5");
        ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] This is last index");
        get_wp = using_func_vec[current_index];
        target_change_dist = target_change_dist_*wp_goal_judg_threshold_ratio_;
        last_flag = true;
        return true;
      }else{
        // ERROR
        ROS_WARN(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] THERE IS AN INVALID START INDEX!!!!");
        return false;
      }
    }else{
      // equivalent...pattern.5
      ROS_INFO(" [AMAMain][SubThread:WayPointThread][Func:get_waypoint_absolute] Get Way Point by Pattern 5");
      get_wp = using_func_vec[current_index];
      return true;
    }
  }

  ama_struct::GetPoint AMAMain::get_origin_position(const ros::Time& original_time, const std::string& original_frame, const std::string& destination_frame)
  {
    tf::StampedTransform transform;
  	while(ros::ok())
  	{
  		try
  		{
        tf_.waitForTransform(destination_frame, original_frame, original_time, ros::Duration(10.0) );
        tf_.lookupTransform(destination_frame, original_frame, original_time, transform);
  		}
  		catch (tf::TransformException &ex)
  		{
        ROS_ERROR("%s",ex.what());
        continue;
  		}
  		break;
  	}
    ama_struct::GetPoint return_gp(destination_frame, original_time);
    return_gp.set_position( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    return_gp.set_orientation( transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
  	return return_gp;
  }

  // get original pose position on the destination_frame coorfinate
  ama_struct::GetPoint AMAMain::get_origin_position_now(const std::string& original_frame, const std::string& destination_frame)
  {
    return get_origin_position(ros::Time(0), original_frame, destination_frame);
  }


  bool AMAMain::loadAMAFuncs(ros::NodeHandle node, const visualization_msgs::Marker& points)
  {
    ama_funcs_map_.clear();
    //!!! YOU SHOULD ARRANGE MOVE_MODES IN ORDER OF YOUR PRIORITY!!!!
    XmlRpc::XmlRpcValue mode_list;
    if(node.getParam("ama_funcs", mode_list)){
      if(mode_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          if(mode_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(mode_list[i].hasMember("name") && mode_list[i].hasMember("type")){
              //同じ名前の機能が定義されていないかチェック
              for(unsigned int j = i + 1; j < mode_list.size(); j++){
                if(mode_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(mode_list[j].hasMember("name") && mode_list[j].hasMember("type")){
                    std::string name_i = mode_list[i]["name"];
                    std::string name_j = mode_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] A ama mode with the name %s already exists, this is not allowed. Using the default ama modes instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }else{
              ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] ama modes must have a name and a type and this does not. Using the default ama modes instead.");
              return false;
            }
          }else{
            ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] ama modes must be specified as maps, but they are XmlRpcType %d. We'll use the default ama modes instead.",
                mode_list[i].getType());
            return false;
          }
        }

        //ここからインスタンス化を行っていく
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          try{
            //使えないクラスがないかチェック
            if(!ama_funcs_loader_.isClassAvailable(mode_list[i]["type"])){
              std::vector<std::string> classes = ama_funcs_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(mode_list[i]["type"] == ama_funcs_loader_.getName(classes[i])){
                  //名前の変更を促す
                  ROS_WARN(" [AMAMain][Func:loadAMAFuncs] ama mode specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(mode_list[i]["type"]).c_str(), classes[i].c_str());
                  mode_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<ama_core::AMAFunc> func(ama_funcs_loader_.createInstance(mode_list[i]["type"]));

            //使える状態かチェック
            if(func.get() == NULL){
              ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            ROS_INFO(" [AMAMain][Func:loadAMAFuncs] Will load this Func-plugin '%s' ", ama_funcs_loader_.getName(mode_list[i]["type"]).c_str());
            func->initialize( ama_funcs_loader_.getName(mode_list[i]["type"]), &tf_);
            ama_funcs_map_[mode_list[i]["name"]] = func;
            eachfunc_points_[mode_list[i]["name"]] = points;
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] Failed to load a Func-plugin. Using default ama modes. Error: %s", ex.what());
            return false;
          }
        }
        if( ama_funcs_map_.count(default_str_) == 0 ){
          // //first, we'll load a move_mode to mode_stop_and_go
          ROS_INFO(" [AMAMain][Func:loadAMAFuncs] Load Default Func-plugin");
          boost::shared_ptr<ama_core::AMAFunc> func_default(ama_funcs_loader_.createInstance("func_default/FuncDefault"));
          func_default->initialize( ama_funcs_loader_.getName("func_default/FuncDefault"), &tf_);
          ama_funcs_map_[default_str_] = func_default;
          eachfunc_points_[default_str_] = points;
        }
      }else{
        ROS_ERROR(" [AMAMain][Func:loadAMAFuncs] The ama modes specification must be a list, but is of XmlRpcType %d. We'll use the default ama modes instead.",
            mode_list.getType());
        return false;
      }
    }else{
      //もし設定がされていなければ,defaultmodeを読み込む
      return false;
    }

    //defaultmodeではないmodeを読み込めた状態
    ROS_INFO(" [AMAMain][Func:loadAMAFuncs] Success loading Func-plugin");
    return true;
  }

  void AMAMain::loadDefaultFunc(const visualization_msgs::Marker& points)
  {
    ama_funcs_map_.clear();
    try{
      ROS_INFO(" [AMAMain][loadDefaultFunc] Load Default Func-plugin");
      boost::shared_ptr<ama_core::AMAFunc> func_default(ama_funcs_loader_.createInstance("func_default/FuncDefault"));
      func_default->initialize( ama_funcs_loader_.getName("func_default/FuncDefault"), &tf_);
      ama_funcs_map_[default_str_] = func_default;
      eachfunc_points_[default_str_] = points;
    }catch(pluginlib::PluginlibException& ex){
      ROS_FATAL(" [AMAMain] Failed to load a plugin. This should not happen on default ama modes. Error: %s", ex.what());
    }
  }

  bool AMAMain::loadAMAMoves(ros::NodeHandle node)
  {
    ama_moves_map_.clear();
    //!!! YOU SHOULD ARRANGE MOVE_MODES IN ORDER OF YOUR PRIORITY!!!!
    XmlRpc::XmlRpcValue mode_list;
    if(node.getParam("ama_moves", mode_list)){
      if(mode_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          if(mode_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(mode_list[i].hasMember("name") && mode_list[i].hasMember("type")){
              //同じ名前の機能が定義されていないかチェック
              for(unsigned int j = i + 1; j < mode_list.size(); j++){
                if(mode_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(mode_list[j].hasMember("name") && mode_list[j].hasMember("type")){
                    std::string name_i = mode_list[i]["name"];
                    std::string name_j = mode_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR(" [AMAMain][Func:loadAMAMoves] A ama move with the name %s already exists, this is not allowed. Using the default ama moves instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }else{
              ROS_ERROR(" [AMAMain][Func:loadAMAMoves] ama moves must have a name and a type and this does not. Using the default ama moves instead.");
              return false;
            }
          }else{
            ROS_ERROR(" [AMAMain][Func:loadAMAMoves] ama moves must be specified as maps, but they are XmlRpcType %d. We'll use the default ama moves instead.",
                mode_list[i].getType());
            return false;
          }
        }

        //ここからインスタンス化を行っていく
        for(unsigned int i = 0; i < mode_list.size(); ++i){
          try{
            //使えないクラスがないかチェック
            if(!ama_moves_loader_.isClassAvailable(mode_list[i]["type"])){
              std::vector<std::string> classes = ama_moves_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(mode_list[i]["type"] == ama_moves_loader_.getName(classes[i])){
                  //名前の変更を促す
                  ROS_WARN(" [AMAMain][Func:loadAMAMoves] ama move specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(mode_list[i]["type"]).c_str(), classes[i].c_str());
                  mode_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<ama_core::AMAMove> mode(ama_moves_loader_.createInstance(mode_list[i]["type"]));

            //使える状態かチェック
            if(mode.get() == NULL){
              ROS_ERROR(" [AMAMain][Func:loadAMAMoves] The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //初期化する
            // 20190815 or 16から以下を書き進める.初期化の仕方を定義.加えて,defaultloadをmove_baseに習って作成
            // mode->initialize(mode_list[i]["name"]);
            ROS_INFO(" [AMAMain][Func:loadAMAMoves] Will load this Move-plugin '%s' ", ama_moves_loader_.getName(mode_list[i]["type"]).c_str());

            mode->initialize( ama_moves_loader_.getName(mode_list[i]["type"]), &movegoal_next_g_, &movegoal_flag_,  &break_flag_, &aborted_flag_, &exinstruct_flag_);
            ama_moves_map_[mode_list[i]["name"]] = mode;
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR(" [AMAMain][Func:loadAMAMoves] Failed to load a Move-plugin. Using default ama moves. Error: %s", ex.what());
            return false;
          }
        }
        if( ama_moves_map_.count(default_str_) == 0 ){
          ROS_INFO(" [AMAMain][Func:loadAMAMoves] Load Default Move-plugin");
          // //first, we'll load a move_mode to mode_stop_and_go
          boost::shared_ptr<ama_core::AMAMove> move_default(ama_moves_loader_.createInstance("move_default/MoveDefault"));
          move_default->initialize( ama_moves_loader_.getName("move_default/MoveDefault") , &movegoal_next_g_, &movegoal_flag_,  &break_flag_, &aborted_flag_, &exinstruct_flag_);
          ama_moves_map_[default_str_] = move_default;
        }
      }else{
        ROS_ERROR(" [AMAMain][Func:loadAMAMoves] The ama moves specification must be a list, but is of XmlRpcType %d. We'll use the default ama moves instead.",
            mode_list.getType());
        return false;
      }
    }else{
      //もし設定がされていなければ,defaultmodeを読み込む
      return false;
    }

    //defaultmodeではないmodeを読み込めた状態
    ROS_INFO(" [AMAMain][Func:loadAMAMoves] Success loading Move-plugin");
    return true;
  }

  void AMAMain::loadDefaultMove(void)
  {
    ama_moves_map_.clear();
    try{
      ROS_INFO(" [AMAMain][Func:loadDefaultMove] Load Default Move-plugin");
      // //first, we'll load a move_mode to mode_stop_and_go
      boost::shared_ptr<ama_core::AMAMove> move_default(ama_moves_loader_.createInstance("move_default/MoveDefault"));
      move_default->initialize( ama_moves_loader_.getName("move_default/MoveDefault"), &movegoal_next_g_, &movegoal_flag_,  &break_flag_, &aborted_flag_, &exinstruct_flag_);
      ama_moves_map_[default_str_] = move_default;

    }catch(pluginlib::PluginlibException& ex){
      ROS_FATAL(" [AMAMain][Func:loadDefaultMove] Failed to load a Move-plugin. This should not happen on default ama moves. Error: %s", ex.what());
    }
  }
};
