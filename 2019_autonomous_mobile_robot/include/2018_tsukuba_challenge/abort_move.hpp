#ifndef _ABORTHEADER_
#define _ABORTHEADER_

#include <Mlib/libfm.hpp>
#include <2018_tsukuba_challenge/scan_class.hpp>
#include <2018_tsukuba_challenge/param_class.hpp>

#include <geometry_msgs/Twist.h>

typedef enum NOW_GOAL_MODE {NOW_MOVE, FINISH} NOW_GOAL_MODE;

class abort_move
{
public:
  abort_move();
  virtual ~abort_move();
  bool mover();
  // reset処理
  bool reset(const NOW_GOAL_MODE& current_state);
private:
  // 前に行くために方向を変更する処理
  void Go_Front(const std::pair<float, float>& root) const;
  // 前に行くために方向を変更する処理
  void Go_Back(const std::pair<float, float>& root) const;
  // 前に行くために方向を変更する処理
  void Stay_Now() const;
  // cmd_velを送ってくれる
  void cmd_vel_pub(geometry_msgs::Twist& cmd_vel_value, const float& time_value) const;
  // cmd_velをセットする
  void set_cmd_vel(geometry_msgs::Twist& set_vel_value, const float& Radian_value , const MOVE_DECIDE& now_mode) const;

  // 初期化勢
  ros::NodeHandle cmd_node_;
  mutable int stay_count_;
  float default_m_obs_, m_obs_front_, m_obs_back_;

  ros::Publisher cmd_pub_;
  std::string scan_topic_;
  float kuaro_linear_, kuaro_angular_, kuaro_back_range_;


  Decisions get_Decision_;
  std::shared_ptr< scan_class > scan_node_;
  std::shared_ptr< param_class > param_node_;

};

#endif
