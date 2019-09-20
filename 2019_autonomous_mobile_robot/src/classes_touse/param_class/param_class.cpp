#include <2018_tsukuba_challenge/param_class.hpp>

param_class::param_class():
min_obs_str_("rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist "), now_min_obstacle_value_(0.0f)
{
  std::cout << " PARAM CLASS READY..... " << std::endl;
}

param_class::~param_class()
{

}

// tebローカルプランナーの min_obstacle_dist をここで設定する
void param_class::min_obstacle_set(const float& set_value)
{
  if( now_min_obstacle_value_ == set_value)
  {
    return;
  }
  now_min_obstacle_value_ = set_value;
  std::string tmp_str = min_obs_str_ + std::to_string(set_value);
  system(tmp_str.c_str());
}



// map上の座標を取得する
bool param_class::Param_get_tf(const std::string& output_frame, const std::string& input_frame, const ros::Time& input_time, tf::StampedTransform& get_trans)
{

	while(ros::ok())
	{
		try
		{
			listener.lookupTransform(output_frame, input_frame, input_time, get_trans);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;

      // return false何かの条件を設けてfalseを出すようにするかも
		}
		break;
	}

	return true;
}

// 自分で入力した地点を任意のフレームでの座標に変換する
bool param_class::Param_get_gm(const std::string& output_frame, const ros::Time& input_time, const geometry_msgs::PoseStamped& input_pose, const std::string& input_frame, geometry_msgs::PoseStamped& gm_pose)
{
  while(ros::ok()){
    try
    {
      listener.transformPose(output_frame, input_time, input_pose, input_frame, gm_pose);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;

      // return false何かの条件を設けてfalseを出すようにするかも
    }
    break;
  }
  return true;
}
