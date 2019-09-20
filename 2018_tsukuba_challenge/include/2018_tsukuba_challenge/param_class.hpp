#ifndef PARAM_CLASS_NAME
#define PARAM_CLASS_NAME


#include <Mlib/libfm.hpp>
#include <functional>

class param_class
{
public:
  param_class();
  virtual ~param_class();
  //min_obstacle_distを変更する
  void min_obstacle_set(const float& set_value);
  // map上の座標を取得する
  bool Param_get_tf(const std::string& output_frame, const std::string& input_frame, const ros::Time& input_time, tf::StampedTransform& get_trans);
  // 自分で入力した地点を任意のフレームでの座標に変換する
  bool Param_get_gm(const std::string& output_frame, const ros::Time& input_time, const geometry_msgs::PoseStamped& input_pose, const std::string& input_frame, geometry_msgs::PoseStamped& gm_pose);



private:
  const std::string min_obs_str_;
  float now_min_obstacle_value_;
  tf::TransformListener listener;

};

#endif
