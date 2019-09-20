/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/30
* <version>		v2.1
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "stdafx.hpp"

struct GetPoint
{
	double position[3];
	double orientation[4];
};

class C_person_manager
{
public:
	C_person_manager(void);
	~C_person_manager(void);

	void get_person_callback(const geometry_msgs::PointStamped::ConstPtr& sub_point);
	void get_scan_back_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

	void min_obstacle_set(int goal_index);
	tf::StampedTransform get_tf(std::string input_frame, std::string output_frame, ros::Time input_time);
	geometry_msgs::PoseStamped get_gm(geometry_msgs::PoseStamped input_pose, std::string input_frame, std::string output_frame, ros::Time input_time);
	void execute_cmd(double trans_vel, double rot_vel, double limit);
	double rounding(double angle);

	void run();

private:
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

	ros::NodeHandle cmd_node;
	ros::Publisher cmd_pub;

	ros::NodeHandle perosn_node;
	ros::Subscriber person_sub;

	ros::NodeHandle scan_node;
	ros::Subscriber scan_sub;
	ros::Publisher cloud_pub;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	laser_geometry::LaserProjection scan_projector;
	tf::TransformListener scan_listener;

	geometry_msgs::PoseStamped gm_pose;

	// launchファイルのパラメータで設定する変数
	std::string read_file;
	double distance;
	double time_limit;
	double back_vel;
	int start_point;
	int map_flag;
	int danger_scan_count;
	double danger_range_x;
	double danger_range_y;

	int goal_index;
	double now_min_obstacle_value;
	std::string min_obs_str;
	std::vector<GetPoint> goal_point;
	std::vector<GetPoint> person_point;
	std::vector<std::pair<int, double>> min_obstacle_setter;

	bool back_danger_flag;

	GetPoint now_person;
	ros::Time person_time;
	bool person_flag;

};
