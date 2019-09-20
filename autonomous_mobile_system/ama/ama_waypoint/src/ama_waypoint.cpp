#include <ama_waypoint/ama_waypoint.hpp>

namespace ama_waypoint{

  AMAWayPoint::AMAWayPoint()
  {
  }

  AMAWayPoint::~AMAWayPoint()
  {

  }


  // WayPointThreadについては,ここで一旦記述
  void AMAMain::WayPointThread(void)
  {
    // param
    ros::NodeHandle n;
    ros::Rate wpget_loop(5);
    ros::Rate wpmg_loop(10);
    std::string uf_name;
    GetPoint current_goal();
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
      const unsigned int& wp_nf_index = wp_nextfunc_vec_[uf_name]->first;
      const std::string& wp_nf_name = wp_nextfunc_vec_[uf_name]->second;

      // PHASE USING WAYPOINT VECTOR
      ROS_INFO(" [AMAMain][SubThread:WayPointThread] PHASE GETTING NEXT WAYPOINT...First");
      // give First waypoint
      // get current "/base_link" position on the "/map"
      current_pose_ = get_origin_position_now();
      // get waypoint. false means an error has occured
      proccesing2get_waypoint(current_goal, target_change_dist, using_func_vec, start_index, last_flag,  uf_name);

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
            break;
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

void AMAMain::proccesing2get_waypoint(ama_core::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag, const std::string& using_func_name )
{
  if( !get_waypoint(get_wp, target_change_dist, using_func_vec, current_index) ){
    // This error occurs when there is an invalid current_index
    ROS_WARN(" [AMAMain][SubThread:WayPointThread][Func:proccesing2get_waypoint] Index Error... Func '%s' MaxIndex '%d', StartIndex '%d", using_func_name.c_str(), using_func_vec.size(), current_index);
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


bool AMAMain::get_waypoint(ama_core::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag)
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
      GetPoint get_wp2 = using_func_vec[current_index+1].differ(get_wp);
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


// get original pose position on the destination_frame coorfinate
ama_core::GetPoint AMAMain::get_origin_position_now(const std::string& original_frame, const std::string& destination_frame)
{
  return get_origin_position(ros::Time(0), original_frame, destination_frame);
}

ama_core::GetPoint AMAMain::get_origin_position(const std::string& original_frame, const std::string& destination_frame, const ros::Time& original_time)
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
  ama_core::GetPoint return_gp(destination_frame, original_time);
  return_gp.set_position( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  return_gp.set_orientation( transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
	return return_gp;
}


// Convert my pose to destination coordinates
geometry_msgs::PoseStamped AMAMain::transform_coorfinatesystem_now(const ama_core::GetPoint& original_pose, const std::string& original_frame, const std::string& destination_frame)
{
	return transform_coorfinatesystem(original_pose, ros::Time(0), original_frame, destination_frame);
}

// Convert my pose to destination coordinates
geometry_msgs::PoseStamped AMAMain::transform_coorfinatesystem(const ama_core::GetPoint& original_pose, const std::string& original_frame, const std::string& destination_frame, const ros::Time& original_time)
{
  geometry_msgs::PoseStamped frame_pose;
	while(ros::ok())
	{
    try {
      tf_.waitForTransform(destination_frame, original_frame, original_time, ros::Duration(10.0) );
      tf_.transformPose(destination_frame, original_time, original_pose, original_frame, frame_pose);

      break;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      continue;
    }
		break;
	}
	return frame_pose;
}




};
