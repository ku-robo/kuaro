#ifndef _RGDHEADER_
#define _RGDHEADER_

#include <Mlib/libfm.hpp>
#include <2018_tsukuba_challenge/abort_move.hpp>

#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef enum WAY_POINTS {POSI_X, POSI_Y, POSI_Z, ORIEN_X, ORIEN_Y, ORIEN_Z, ORIEN_W} WAY_POINTS;//POSI = position ... ORIEN = orientation
typedef enum MND_MODE {} MND_MODE;//MND = min_obstacle_distのパラメータをmodeによって変更するため

const int data_num = 7;//Get_pointの内部のデータ数
const int main_rate = 30;

const std::string	min_obs_str = "rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS min_obstacle_dist ";//

struct Rgd_Params
{
  // launchファイルのパラメータで設定する変数
  std::string Client_name;
  std::string read_file;
  double distance;
  double time_limit;
  double back_vel;
  int start_point;
  int map_flag;
  int danger_scan_count;
  double danger_range_x;
  double danger_range_y;

  void print_member() const{
    std::cout << "Client_name:   " << Client_name << std::endl;
    std::cout << "read_file:   " << read_file << std::endl;
    std::cout << "distance:   " << distance << std::endl;
    std::cout << "time_limit:   " << time_limit << std::endl;
    std::cout << "back_vel:   " << back_vel << std::endl;
    std::cout << "start_point:   " << start_point << std::endl;
    std::cout << "map_flag:   " << map_flag << std::endl;
    std::cout << "danger_scan_count:   " << danger_scan_count << std::endl;
    std::cout << "danger_range_x:   " << danger_range_x << std::endl;
    std::cout << "danger_range_y:   " << danger_range_y << std::endl;
  };
};

struct Get_point
{
  double data[data_num];
};

//lib用のクラスで便利そーな関数がはいっとる
class Robot_goals_decider
{
public:
  Robot_goals_decider();
  virtual ~Robot_goals_decider();
  bool main_mover();

private:
//初期化勢
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  bool person_flag_,	back_danger_flag_;
  int goal_index_,now_try_count_;
	double now_min_obstacle_value_;
  abort_move ab_c_;
  NOW_GOAL_MODE goal_states;
  //launch paramを列挙
  ros::NodeHandle nh;
//初期化しない勢
  Rgd_Params Rgd_Param_const_;
  std::unique_ptr<MoveBaseClient> using_Client_;
  std::vector< Get_point > waypoint_box_;
  std::vector< Get_point > person_point_;
  move_base_msgs::MoveBaseGoal main_goal_, person_goal_;
  geometry_msgs::PoseStamped gm_input_pose_, gm_output_pose_;


// 値を入れたり,関数を使うためだけの箱
  tf::TransformListener listener_;

  laser_geometry::LaserProjection scan_projector;
  tf::TransformListener scan_listener_;

  ros::Time person_time_;

//関数勢
  //人を検出しなかった場合の動き
  bool MainGoal_mode(const std::unique_ptr<MoveBaseClient>& ac, const move_base_msgs::MoveBaseGoal& goal, const Get_point& now_point, bool current_state);
  //人を検出した際の動き
  bool PersonGoal_mode();
  // map上の座標を取得する
  tf::StampedTransform get_tf(const std::string& output_frame, const std::string& input_frame, const ros::Time &input_time);
  // 自分で入力した地点を任意のフレームでの座標に変換する
  geometry_msgs::PoseStamped get_gm(const std::string& output_frame, const ros::Time &input_time, const geometry_msgs::PoseStamped& input_pose, const std::string& input_frame);
  // 人物データを拾うCallback関数
  void get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point);
  //　csvを読み込んで落としこむ
  bool csv_reader(std::vector< Get_point >& data_box, const std::string& path);
  void waypoints_inputer(move_base_msgs::MoveBaseGoal& goal, const std::string& frame_id, const Get_point& using_data);//goal用......値を代入するためだけの関数
  void waypoints_inputer(move_base_msgs::MoveBaseGoal& goal, const std::string& frame_id, const Get_point& using_data, const geometry_msgs::Quaternion& Q_point);//person用......値を代入するためだけの関数
  void waypoints_inputer(geometry_msgs::PoseStamped& gm_input_pose, const std::string& frame_id, const Get_point& using_data);//goal用......値を代入するためだけの関数
  void waypoints_inputer(geometry_msgs::PoseStamped& gm_input_pose, const ros::Time& person_time, const std::string& frame_id, const Get_point& using_data);//person用......値を代入するためだけの関数

};

#endif
