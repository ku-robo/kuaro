#include <ama_struct/get_point.hpp>

namespace ama_struct{

  // Global
  GetPoint operator+(const GetPoint& gp1, const GetPoint& gp2)
  {
    return GetPoint(gp1) += gp2;
  }

  GetPoint operator+(const move_base_msgs::MoveBaseGoal& goal1, const GetPoint& gp2)
  {
    return GetPoint(goal1) += gp2;
  }

  GetPoint operator+(const GetPoint& gp1, const move_base_msgs::MoveBaseGoal& goal2)
  {
    return GetPoint(gp1) += goal2;
  }

  GetPoint operator+(const geometry_msgs::PoseStamped& g_pose1, const GetPoint& gp2)
  {
    return GetPoint(g_pose1) += gp2;
  }

  GetPoint operator+(const GetPoint& gp1, const geometry_msgs::PoseStamped& g_pose2)
  {
    return GetPoint(gp1) += g_pose2;
  }


  GetPoint operator-(const GetPoint& gp1, const GetPoint& gp2)
  {
    return GetPoint(gp1) -= gp2;
  }

  GetPoint operator-(const move_base_msgs::MoveBaseGoal& goal1, const GetPoint& gp2)
  {
    return GetPoint(goal1) -= gp2;
  }

  GetPoint operator-(const GetPoint& gp1, const move_base_msgs::MoveBaseGoal& goal2)
  {
    return GetPoint(gp1) -= goal2;
  }

  GetPoint operator-(const geometry_msgs::PoseStamped& g_pose1, const GetPoint& gp2)
  {
    return GetPoint(g_pose1) -= gp2;
  }

  GetPoint operator-(const GetPoint& gp1, const geometry_msgs::PoseStamped& g_pose2)
  {
    return GetPoint(gp1) -= g_pose2;
  }

  GetPoint::GetPoint(void)
  {
    create_empty();
  }

  GetPoint::GetPoint(const std::string& thisframe_id)
  {
    set_other(thisframe_id, ros::Time(0));
    create_equivalence(0.0);
  }

  GetPoint::GetPoint(const std::string& thisframe_id, const ros::Time& yourtime)
  {
    set_other(thisframe_id, yourtime);
    create_equivalence(0.0);
  }

  GetPoint::GetPoint(const move_base_msgs::MoveBaseGoal& goal)
  {
    set(goal);
  }

  GetPoint::GetPoint(const move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime)
  {
    set(goal, thisframe_id, yourtime);
  }

  GetPoint::GetPoint(const geometry_msgs::PoseStamped& g_pose)
  {
    set(g_pose);
  }

  GetPoint::GetPoint(const geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime)
  {
    set(g_pose, thisframe_id, yourtime);
  }

  GetPoint::GetPoint(const GetPoint& gp)
  {
    frame_id = gp.frame_id;
    stamp = gp.stamp;
    position_x = gp.position_x;
    position_y = gp.position_y;
    position_z = gp.position_z;
    orientation_x = gp.orientation_x;
    orientation_y = gp.orientation_y;
    orientation_z = gp.orientation_z;
    orientation_w = gp.orientation_w;
  }

  GetPoint::~GetPoint(){}

  void GetPoint::create_empty(void)
  {
    set_other("/base_link", ros::Time::now());
    create_equivalence(0.0);
  }

  void GetPoint::create_equivalence( const float& equivalence )
  {
    allset_position(equivalence);
    allset_orientation(equivalence);
  }

  void GetPoint::set_other(const std::string& thisframe_id, const ros::Time& yourtime)
  {
    frame_id = thisframe_id;
    stamp = yourtime;
  }

  void GetPoint::allset_position( const float& equivalence )
  {
    position_x = equivalence;
    position_y = equivalence;
    position_z = equivalence;
  }

  void GetPoint::set_position( const float& x_val, const float& y_val, const float& z_val)
  {
    position_x = x_val;
    position_y = y_val;
    position_z = z_val;
  }

  void GetPoint::allset_orientation( const float& equivalence )
  {
    orientation_x = equivalence;
    orientation_y = equivalence;
    orientation_z = equivalence;
    orientation_w = equivalence;
  }

  void GetPoint::set_orientation( const float& x_val, const float& y_val, const float& z_val, const float& w_val)
  {
    orientation_x = x_val;
    orientation_y = y_val;
    orientation_z = z_val;
    orientation_w = w_val;
  }

  float GetPoint::gp_atan2(const float& y_val, const float& x_val)
  {
    if(position_x == 0.0 && 0.0 < position_y){
      return M_PI/2.;
    }else if(position_x == 0.0 && position_y < 0.0){
      return -M_PI/2.;
    }else if(position_x == 0.0 && position_y == 0.0){
      return 0.0;
    }
    return std::atan2(y_val, x_val);
  }


  void GetPoint::set_orientation_Yonly_byXY(const float& x_val, const float& y_val)
  {
    set_orientation_Yonly( gp_atan2(y_val, x_val) );
  }

  void GetPoint::set_orientation_Yonly_byPosiiton(void)
  {
    set_orientation_Yonly_byXY(position_x, position_y);
  }

  float GetPoint::InverseYaw(void)
  {
    return gp_atan2(-position_y, -position_x);
  }

  void GetPoint::set_orientation_InverseYonly_byXY(const float& x_val, const float& y_val)
  {
    set_orientation_Yonly( gp_atan2(-y_val, -x_val) );
  }

  void GetPoint::set_orientation_InverseYonly_byPosiiton(void)
  {
    set_orientation_InverseYonly_byXY(position_x, position_y);
  }

  void GetPoint::set_orientation_Yonly(const float& yaw_val)
  {
    set_orientation_RPY(0.0, 0.0, yaw_val);
  }

  void GetPoint::set_orientation_RPY( const float& roll_val, const float& pitch_val, const float& yaw_val)
  {
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(roll_val, pitch_val, yaw_val);
    yaw = yaw_val;
    orientation_x = quaternion.x;
    orientation_y = quaternion.y;
    orientation_z = quaternion.z;
    orientation_w = quaternion.w;
  }

  void GetPoint::set(const move_base_msgs::MoveBaseGoal& goal)
  {
    frame_id = goal.target_pose.header.frame_id;
    stamp = goal.target_pose.header.stamp;
    position_x = goal.target_pose.pose.position.x;
    position_y = goal.target_pose.pose.position.y;
    position_z = goal.target_pose.pose.position.z;
    orientation_x = goal.target_pose.pose.orientation.x;
    orientation_y = goal.target_pose.pose.orientation.y;
    orientation_z = goal.target_pose.pose.orientation.z;
    orientation_w = goal.target_pose.pose.orientation.w;
  }

  void GetPoint::set(const move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime)
  {
    frame_id = thisframe_id;
    stamp = yourtime;
    position_x = goal.target_pose.pose.position.x;
    position_y = goal.target_pose.pose.position.y;
    position_z = goal.target_pose.pose.position.z;
    orientation_x = goal.target_pose.pose.orientation.x;
    orientation_y = goal.target_pose.pose.orientation.y;
    orientation_z = goal.target_pose.pose.orientation.z;
    orientation_w = goal.target_pose.pose.orientation.w;
  }

  void GetPoint::set(const geometry_msgs::PoseStamped& g_pose)
  {
    frame_id = g_pose.header.frame_id;
    stamp = g_pose.header.stamp;
    position_x = g_pose.pose.position.x;
    position_y = g_pose.pose.position.y;
    position_z = g_pose.pose.position.z;
    orientation_x = g_pose.pose.orientation.x;
    orientation_y = g_pose.pose.orientation.y;
    orientation_z = g_pose.pose.orientation.z;
    orientation_w = g_pose.pose.orientation.w;
  }

  void GetPoint::set(const geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime)
  {
    frame_id = thisframe_id;
    stamp = yourtime;
    position_x = g_pose.pose.position.x;
    position_y = g_pose.pose.position.y;
    position_z = g_pose.pose.position.z;
    orientation_x = g_pose.pose.orientation.x;
    orientation_y = g_pose.pose.orientation.y;
    orientation_z = g_pose.pose.orientation.z;
    orientation_w = g_pose.pose.orientation.w;
  }

  void GetPoint::get(move_base_msgs::MoveBaseGoal& goal) const
  {
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = stamp;
    goal.target_pose.pose.position.x = position_x;
    goal.target_pose.pose.position.y = position_y;
    goal.target_pose.pose.position.z = position_z;
    goal.target_pose.pose.orientation.x = orientation_x;
    goal.target_pose.pose.orientation.y = orientation_y;
    goal.target_pose.pose.orientation.z = orientation_z;
    goal.target_pose.pose.orientation.w = orientation_w;
  }

  void GetPoint::get(move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime) const
  {
    goal.target_pose.header.frame_id = thisframe_id;
    goal.target_pose.header.stamp = yourtime;
    goal.target_pose.pose.position.x = position_x;
    goal.target_pose.pose.position.y = position_y;
    goal.target_pose.pose.position.z = position_z;
    goal.target_pose.pose.orientation.x = orientation_x;
    goal.target_pose.pose.orientation.y = orientation_y;
    goal.target_pose.pose.orientation.z = orientation_z;
    goal.target_pose.pose.orientation.w = orientation_w;
  }

  void GetPoint::get(geometry_msgs::PoseStamped& g_pose) const
  {
    g_pose.header.frame_id = frame_id;
    g_pose.header.stamp = stamp;
    g_pose.pose.position.x = position_x;
    g_pose.pose.position.y = position_y;
    g_pose.pose.position.z = position_z;
    g_pose.pose.orientation.x = orientation_x;
    g_pose.pose.orientation.y = orientation_y;
    g_pose.pose.orientation.z = orientation_z;
    g_pose.pose.orientation.w = orientation_w;
  }

  void GetPoint::get(geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime) const
  {
    g_pose.header.frame_id = thisframe_id;
    g_pose.header.stamp = yourtime;
    g_pose.pose.position.x = position_x;
    g_pose.pose.position.y = position_y;
    g_pose.pose.position.z = position_z;
    g_pose.pose.orientation.x = orientation_x;
    g_pose.pose.orientation.y = orientation_y;
    g_pose.pose.orientation.z = orientation_z;
    g_pose.pose.orientation.w = orientation_w;
  }

  geometry_msgs::PoseStamped GetPoint::geometryPoseStamped(void) const
  {
    geometry_msgs::PoseStamped g_pose;
    get(g_pose);
    return g_pose;
  }

  geometry_msgs::PoseStamped GetPoint::geometryPoseStamped(const std::string& thisframe_id, const ros::Time& yourtime) const
  {
    geometry_msgs::PoseStamped g_pose;
    get(g_pose, thisframe_id, yourtime);
    return g_pose;
  }

  move_base_msgs::MoveBaseGoal GetPoint::MoveBaseGoal(void) const
  {
    move_base_msgs::MoveBaseGoal goal;
    get(goal);
    return goal;
  }

  move_base_msgs::MoveBaseGoal GetPoint::MoveBaseGoal(const std::string& thisframe_id, const ros::Time& yourtime) const
  {
    move_base_msgs::MoveBaseGoal goal;
    get(goal, thisframe_id, yourtime);
    return goal;
  }


  GetPoint GetPoint::differ(const GetPoint& gp) const
  {
    GetPoint differ_val;
    differ_val = (*this) - gp;
    differ_val.set_orientation_Yonly_byPosiiton();
    return differ_val;
  }

  GetPoint GetPoint::differ(const geometry_msgs::PoseStamped& gmps) const
  {
    GetPoint differ_val;
    differ_val = (*this) - gmps;
    differ_val.set_orientation_Yonly_byPosiiton();
    return differ_val;
  }

  GetPoint GetPoint::differ(const move_base_msgs::MoveBaseGoal& mbg) const
  {
    GetPoint differ_val;
    differ_val = (*this) - mbg;
    differ_val.set_orientation_Yonly_byPosiiton();
    return differ_val;
  }

  geometry_msgs::PoseStamped GetPoint::differ_geometryPoseStamped(const GetPoint& gp)
  {
    return differ(gp).geometryPoseStamped();
  }

  geometry_msgs::PoseStamped GetPoint::differ_geometryPoseStamped(const geometry_msgs::PoseStamped& gmps)
  {
    return differ(gmps).geometryPoseStamped();
  }

  geometry_msgs::PoseStamped GetPoint::differ_geometryPoseStamped(const move_base_msgs::MoveBaseGoal& mbg )
  {
    return differ(mbg).geometryPoseStamped();
  }

  move_base_msgs::MoveBaseGoal GetPoint::differ_MoveBaseGoal(const GetPoint& gp)
  {
    return differ(gp).MoveBaseGoal();
  }

  move_base_msgs::MoveBaseGoal GetPoint::differ_MoveBaseGoal(const geometry_msgs::PoseStamped& gmps)
  {
    return differ(gmps).MoveBaseGoal();
  }

  move_base_msgs::MoveBaseGoal GetPoint::differ_MoveBaseGoal(const move_base_msgs::MoveBaseGoal& mbg)
  {
    return differ(mbg).MoveBaseGoal();
  }

  float GetPoint::xy_hypot(void)
  {
    return std::hypot(position_x, position_y);
  }

  float GetPoint::xy_hypot(const GetPoint& gp)
  {
    return differ(gp).xy_hypot();
  }

  float GetPoint::xy_hypot(const geometry_msgs::PoseStamped& gmps)
  {
    return differ(gmps).xy_hypot();
  }

  float GetPoint::xy_hypot(const move_base_msgs::MoveBaseGoal& mbg )
  {
    return differ(mbg).xy_hypot();
  }

  GetPoint& GetPoint::xy_flexibility(const float& distination_hypot)
  {
    return xy_ratio_flexibility(distination_hypot/xy_hypot());
  }

  GetPoint& GetPoint::xy_flexibility(const float& original_hypot, const float& distination_hypot)
  {
    float ratio;
    if( original_hypot == 0.0f){
      ratio = 0.0;
    }else{
      ratio = distination_hypot/original_hypot;
    }
    return xy_ratio_flexibility(ratio);
  }

  GetPoint& GetPoint::xy_ratio_flexibility(const float& ratio)
  {
    if( 0.0 < ratio ){
      position_x = position_x*ratio;
      position_y = position_y*ratio;
      set_orientation_Yonly_byPosiiton();
    }else if( ratio == 0.0){
      position_x = 0.0;
      position_y = 0.0;
    }else{
      position_x = position_x*ratio;
      position_y = position_y*ratio;
      set_orientation_InverseYonly_byPosiiton();
    }
    return *this;
  }


  GetPoint& GetPoint::operator =(const GetPoint& gp) &
  {
    frame_id = gp.frame_id;
    stamp = gp.stamp;
    position_x = gp.position_x;
    position_y = gp.position_y;
    position_z = gp.position_z;
    orientation_x = gp.orientation_x;
    orientation_y = gp.orientation_y;
    orientation_z = gp.orientation_z;
    orientation_w = gp.orientation_w;
    return *this;
  }

  GetPoint& GetPoint::operator =(const GetPoint&& gp) & noexcept
  {
    frame_id = std::move(gp.frame_id);
    stamp =std::move(gp.stamp);
    position_x = std::move(gp.position_x);
    position_y = std::move(gp.position_y);
    position_z = std::move(gp.position_z);
    orientation_x = std::move(gp.orientation_x);
    orientation_y = std::move(gp.orientation_y);
    orientation_z = std::move(gp.orientation_z);
    orientation_w = std::move(gp.orientation_w);
    return *this;
  }

  GetPoint& GetPoint::operator =(const move_base_msgs::MoveBaseGoal& goal) &
  {
    frame_id = goal.target_pose.header.frame_id;
    stamp = goal.target_pose.header.stamp;
    position_x = goal.target_pose.pose.position.x;
    position_y = goal.target_pose.pose.position.y;
    position_z = goal.target_pose.pose.position.z;
    orientation_x = goal.target_pose.pose.orientation.x;
    orientation_y = goal.target_pose.pose.orientation.y;
    orientation_z = goal.target_pose.pose.orientation.z;
    orientation_w = goal.target_pose.pose.orientation.w;
    return *this;
  }

  GetPoint& GetPoint::operator=(const move_base_msgs::MoveBaseGoal&& goal) & noexcept
  {
    frame_id = std::move(goal.target_pose.header.frame_id);
    stamp = std::move(goal.target_pose.header.stamp);
    position_x = std::move(goal.target_pose.pose.position.x);
    position_y = std::move(goal.target_pose.pose.position.y);
    position_z = std::move(goal.target_pose.pose.position.z);
    orientation_x = std::move(goal.target_pose.pose.orientation.x);
    orientation_y = std::move(goal.target_pose.pose.orientation.y);
    orientation_z = std::move(goal.target_pose.pose.orientation.z);
    orientation_w = std::move(goal.target_pose.pose.orientation.w);
    return *this;
  }

  GetPoint& GetPoint::operator=(const geometry_msgs::PoseStamped& g_pose) &
  {
    frame_id = g_pose.header.frame_id;
    stamp = g_pose.header.stamp;
    position_x = g_pose.pose.position.x;
    position_y = g_pose.pose.position.y;
    position_z = g_pose.pose.position.z;
    orientation_x = g_pose.pose.orientation.x;
    orientation_y = g_pose.pose.orientation.y;
    orientation_z = g_pose.pose.orientation.z;
    orientation_w = g_pose.pose.orientation.w;
    return *this;
  }

  GetPoint& GetPoint::operator=(const geometry_msgs::PoseStamped&& g_pose) & noexcept
  {
    frame_id = std::move(g_pose.header.frame_id);
    stamp = std::move(g_pose.header.stamp);
    position_x = std::move(g_pose.pose.position.x);
    position_y = std::move(g_pose.pose.position.y);
    position_z = std::move(g_pose.pose.position.z);
    orientation_x = std::move(g_pose.pose.orientation.x);
    orientation_y = std::move(g_pose.pose.orientation.y);
    orientation_z = std::move(g_pose.pose.orientation.z);
    orientation_w = std::move(g_pose.pose.orientation.w);
    return *this;
  }

  GetPoint& GetPoint::operator+=(const GetPoint& gp)
  {
    // Do only simple additions
    position_x += gp.position_x;
    position_y += gp.position_y;
    position_z += gp.position_z;
    orientation_x += gp.orientation_x;
    orientation_y += gp.orientation_y;
    orientation_z += gp.orientation_z;
    orientation_w += gp.orientation_w;
    return *this;
  }

  GetPoint& GetPoint::operator+=(const move_base_msgs::MoveBaseGoal& goal)
  {
    // Do only simple additions
    position_x += goal.target_pose.pose.position.x;
    position_y += goal.target_pose.pose.position.y;
    position_z += goal.target_pose.pose.position.z;
    orientation_x += goal.target_pose.pose.orientation.x;
    orientation_y += goal.target_pose.pose.orientation.y;
    orientation_z += goal.target_pose.pose.orientation.z;
    orientation_w += goal.target_pose.pose.orientation.w;
    return *this;
  }

  GetPoint& GetPoint::operator+=(const geometry_msgs::PoseStamped& g_pose)
  {
    // Do only simple additions
    position_x += g_pose.pose.position.x;
    position_y += g_pose.pose.position.y;
    position_z += g_pose.pose.position.z;
    orientation_x += g_pose.pose.orientation.x;
    orientation_y += g_pose.pose.orientation.y;
    orientation_z += g_pose.pose.orientation.z;
    orientation_w += g_pose.pose.orientation.w;
    return *this;
  }

  GetPoint& GetPoint::operator-=(const GetPoint& gp)
  {
    // Do only simple additions
    position_x -= gp.position_x;
    position_y -= gp.position_y;
    position_z -= gp.position_z;
    return *this;
  }

  GetPoint& GetPoint::operator-=(const move_base_msgs::MoveBaseGoal& goal)
  {
    // Do only simple additions
    position_x -= goal.target_pose.pose.position.x;
    position_y -= goal.target_pose.pose.position.y;
    position_z -= goal.target_pose.pose.position.z;
    return *this;
  }

  GetPoint& GetPoint::operator-=(const geometry_msgs::PoseStamped& g_pose)
  {
    // Do only simple additions
    position_x -= g_pose.pose.position.x;
    position_y -= g_pose.pose.position.y;
    position_z -= g_pose.pose.position.z;
    return *this;
  }


};
