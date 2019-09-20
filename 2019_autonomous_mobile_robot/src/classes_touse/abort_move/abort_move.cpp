#include <2018_tsukuba_challenge/abort_move.hpp>

abort_move::abort_move():
cmd_node_("~"), stay_count_(0)
{
  cmd_node_.param("scan_topic", scan_topic_, std::string("/scan"));
  cmd_node_.param("kuaro_linear", kuaro_linear_, 0.7f);//m/s
  cmd_node_.param("kuaro_angular", kuaro_angular_, 0.6f);//rad/s
  cmd_node_.param("kuaro_back_range", kuaro_back_range_, 0.5f);//m
  cmd_node_.param("default_m_obs", default_m_obs_, 0.5f);//defa
  cmd_node_.param("m_obs_front", m_obs_front_, 0.3f);//坂用
  cmd_node_.param("m_obs_back", m_obs_back_, 0.1f);//バック用
  std::cout << "scan_topic : " << scan_topic_ << std::endl;
  std::cout << "kuaro_linear : " << kuaro_linear_ << std::endl;
  std::cout << "kuaro_angular : " << kuaro_angular_ << std::endl;
  std::cout << "kuaro_back_range : " << kuaro_back_range_ << std::endl;
  std::cout << "default_m_obs : " << default_m_obs_ << std::endl;
  std::cout << "m_obs_front : " << m_obs_front_ << std::endl;
  std::cout << "m_obs_back : " << m_obs_back_ << std::endl;
  scan_node_.reset(new scan_class(scan_topic_, 1));
  param_node_.reset(new param_class());
  cmd_pub_ = cmd_node_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

abort_move::~abort_move()
{

}

bool abort_move::mover()
{
  scan_node_->scan_Situation();
  get_Decision_ = scan_node_->Out_Decisions();
  switch (get_Decision_.State) {
    case GO_FRONT:{
      Go_Front(get_Decision_.Root);
      return true;
    }

    case GO_BACK:{
      Go_Back(get_Decision_.Root);
      return true;
    }

    case STAY_NOW:{
        Stay_Now();
        mover();
      return false;
    }
  }
}

// resetするための処理
bool abort_move::reset(const NOW_GOAL_MODE& current_state)
{
  if(current_state == FINISH){
    param_node_->min_obstacle_set(default_m_obs_);
  }
}

// 前に行くために方向を変更する処理
void abort_move::Go_Front(const std::pair<float, float>& root) const//Xは前方方向に↑正,Yは左方向に←正
{
  const float& point_x = root.first;
  const float& point_y = root.second;
  float Radian = std::asin( point_y / hypot( point_x - 0.0f, point_y - 0.0f) );
  geometry_msgs::Twist pub_vel;
  set_cmd_vel(pub_vel, Radian, GO_FRONT);
  float tw_time = 1.25f*std::abs(Radian) / pub_vel.angular.z;
  cmd_vel_pub(pub_vel, tw_time);
  param_node_->min_obstacle_set(m_obs_front_);
}

// 前に行くために方向を変更する処理
void abort_move::Go_Back(const std::pair<float, float>& root) const
{
  const float& point_x = root.first;
  const float& point_y = root.second;
  float Radian = std::asin( point_y / hypot( point_x - 0.0f, point_y - 0.0f) );
  geometry_msgs::Twist pub_vel;
  set_cmd_vel(pub_vel, Radian, GO_BACK);
  float tw_time = 1.25f*std::abs(Radian) / pub_vel.angular.z;
  cmd_vel_pub(pub_vel, tw_time);
  pub_vel.linear.x = -1.0f*kuaro_linear_;
  tw_time = 1.25f*std::abs(kuaro_back_range_) / pub_vel.linear.x;
  cmd_vel_pub(pub_vel, tw_time);
  param_node_->min_obstacle_set(m_obs_back_);
}

void abort_move::set_cmd_vel(geometry_msgs::Twist& set_vel_value, const float& Radian_value , const MOVE_DECIDE& now_mode) const
{
  set_vel_value.linear.x = 0.0;
  set_vel_value.linear.y = 0.0;
  set_vel_value.linear.z = 0.0;
  set_vel_value.angular.x = 0.0;
  set_vel_value.angular.y = 0.0;
  if(now_mode == GO_FRONT){
    if(Radian_value > 0.0f){
      set_vel_value.angular.z = kuaro_angular_;
    }else{
      set_vel_value.angular.z = -1.0f*kuaro_angular_;
    }
    return ;
  }else if(now_mode == GO_BACK){
    if(Radian_value > 0.0f){
      set_vel_value.angular.z = -1.0f*kuaro_angular_;
    }else{
      set_vel_value.angular.z = kuaro_angular_;
    }
    return ;
  }else{
    return ;
  }
}

// ルートがないので待機する処理
void abort_move::Stay_Now() const
{
  if(stay_count_ == 0){
    std::cout << "stay....." <<std::endl;
  }
  if( stay_count_ == 5 ){
    stay_count_ = 0;
  }
  stay_count_++;
}

// cmd_velを送ってくれる
void abort_move::cmd_vel_pub(geometry_msgs::Twist& cmd_vel_value, const float& time_value) const
{

  ros::Rate loop(25);
  ros::Time First_time, Second_time;
  float dt;
  First_time = ros::Time::now();
  while(ros::ok())
  {
    Second_time = ros::Time::now();
    dt = (float)(Second_time - First_time).toSec();
    if( dt >= time_value)
    {
      cmd_vel_value.linear.x = 0.0f;
      cmd_vel_value.angular.z = 0.0f;
      cmd_pub_.publish(cmd_vel_value);
      return ;
    }
  }
}
