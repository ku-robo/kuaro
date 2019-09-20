#include <2018_tsukuba_challenge/scan_class.hpp>


scan_class::scan_class(const std::string& topic, int Q_value, const std::string& name):
sub_core_LS(topic, Q_value, name)
{
  nh_.param("Danger_range", Danger_range_, 1.0f);
  nh_.param("Thre_deg", Thre_deg_, 50.0f);
  nh_.param("Sign_min", Sign_min_, 0.0f);
  nh_.param("Sign_max", Sign_max_, 1.0f);
}

scan_class::~scan_class()
{

}

void scan_class::scan_Situation()
{
  ros::Rate loop(20);
  resub();
  Check_flag_ = false;
  while(ros::ok()){
    Callback_flag_ = false;
    loop.sleep();
    ros::spinOnce();
    if(Callback_flag_){
      endsub();
      break;
    }
  }
  processing(scan_data_);
}

//Decisionsを外部に出力
Decisions scan_class::Out_Decisions() const
{
  return now_Decision_;
}

// 使いやすく処理する
void scan_class::processing(const sensor_msgs::LaserScan& scan_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pub_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	sensor_msgs::PointCloud tmp_cloud1;
	sensor_msgs::PointCloud2 tmp_cloud2;
	tf::TransformListener scan_listener;
  laser_geometry::LaserProjection scan_projector;

	while(ros::ok())
	{
		try
		{
			scan_listener.waitForTransform(scan_msg.header.frame_id.c_str(), scan_msg.header.frame_id.c_str(), scan_msg.header.stamp, ros::Duration(0.1));
			scan_projector.transformLaserScanToPointCloud(scan_msg.header.frame_id, scan_msg, tmp_cloud1, scan_listener);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		break;
	}

	sensor_msgs::convertPointCloudToPointCloud2(tmp_cloud1,tmp_cloud2);
	pcl::fromROSMsg(tmp_cloud2, *scan_cloud);
  Situation_processing(scan_cloud);
}

void scan_class::Situation_processing(const pcl::PointCloud<pcl::PointXYZ>::Ptr using_cloud)
{
  int cloud_size = (int) using_cloud->points.size();
  std::vector< std::pair<float, float> > near_box_front, near_box_back;
	int danger_count = 0;
	for(int i = 0; i < cloud_size; i++)
	{
    float& point_x = using_cloud->points[i].y;//座標系では、yが前方方向),xが横方向なので注意
    float& point_y = using_cloud->points[i].x;// Y は↑,　Xは←の向きが正
    float dist = hypot(point_x - 0.0, point_y- 0.0 );
    if(Danger_range_ >= dist){
      if(point_x > 0.0){
        near_box_front.push_back(std::make_pair(point_x, point_y));
      }else {
        near_box_back.push_back(std::make_pair(point_x, point_y));
      }
      danger_count++;
    }
	}
  calc_processing_forroot(near_box_front, near_box_back);

  std::cout << "Full points num is " << cloud_size << std::endl;
  std::cout << "danger points num is " << danger_count << std::endl;
  std::cout << "front points num is " << near_box_front.size() << std::endl;
  std::cout << "back points num is " << near_box_back.size() << std::endl;
}

// 最終的にどうするかを決めるために色々な処理をしてくれる関数
void scan_class::calc_processing_forroot(std::vector<std::pair<float, float> >& pair_front, std::vector<std::pair<float, float> >& pair_back)
{
  bool front_flag = false, back_flag = false;
  WPairFloat front_root,back_root;
  std::pair<float, float> cout_pair_f, cout_pair_b;
  float front_deg = 0.0f, back_deg = 0.0f;
  std::sort(pair_front.begin(), pair_front.end());
  front_flag = Get_root(front_root, front_deg, pair_front, Sign_max_, Sign_min_);
  cout_pair_f = calc_root(front_root);//画面上の出力は,xが前方,yは左が正....degreeは真正面から何度かを示し,半時計回りに値がおおきくなる
  std::cout << "Go_front : points is (x,y) = (" << cout_pair_f.first << ", " << cout_pair_f.second << ")  width_deg is = " << front_deg << std::endl;
  std::cout << "degree" << std::asin(cout_pair_f.second/pair_hypot(cout_pair_f))*180.0f/M_PI << std::endl;
  std::sort(pair_back.begin(), pair_back.end());
  float defSign_min_ = -1.0*Sign_max_;
  back_flag = Get_root(back_root, back_deg, pair_back, Sign_min_, defSign_min_);
  cout_pair_b = calc_root(back_root);
  std::cout << "Go_back : points is (x,y) = (" << cout_pair_b.first << ", " << cout_pair_b.second << ")  width_deg is = " << back_deg << std::endl;
  std::cout << "degree" << std::asin(cout_pair_b.second/pair_hypot(cout_pair_b))*180.0f/M_PI << std::endl;
  Decisions Decision_box;
  if(front_deg < Thre_deg_ && back_deg < Thre_deg_){
    std::cout << "stay..... " << std::endl;
    Decision_box.State = STAY_NOW;
    Decision_box.Root = std::make_pair(0.0f, 0.0f);
  }else if(front_deg > back_deg){
    std::cout << "Go_front" << std::endl;
    Decision_box.State = GO_FRONT;
    Decision_box.Root = cout_pair_f;
  }else{
    std::cout << "Go_back" << std::endl;
    Decision_box.State = GO_BACK;
    Decision_box.Root = cout_pair_b;
  }
  Set_Decisions(Decision_box);
  return ;
}

inline std::pair<float, float> scan_class::calc_root(const WPairFloat& root)
{
  return std::make_pair( make_Spoint(root.first.first, root.second.first), make_Spoint(root.first.second, root.second.second) );
}

inline float scan_class::make_Spoint(const float& point1, const float& point2)
{
  return (point1 + point2) / 2.0f;
}

// ルートを見つけるための処理...事前にソート処理が必要
bool scan_class::Get_root(WPairFloat& root, float& degree_value, const std::vector<std::pair<float, float> >& pair_data, const float& Sign_max, const float& Sign_min)
{
  float max_front = Sign_max*Danger_range_;
  float min_front = Sign_min*Danger_range_;
  if(pair_data.size() == 0 ){
    if( std::abs(max_front) > std::abs(min_front) ){
      min_front = std::abs(max_front);
    }else{
      min_front = std::abs(min_front);
    }
    root = std::make_pair(std::make_pair(max_front, -1.0*min_front), std::make_pair(max_front, min_front));
    degree_value = 180.0f;
    return true;
  }
  std::vector< WPairFloat > Get_box, insert_box;
  std::vector<std::pair<float, float> > changed_pair;
  bool get_flag = false;
  Get_box = Comp_width(pair_data, max_front, min_front);//先に前方向について処理
  changed_pair = pair_changer(pair_data);//pairのxとyを入れ替えた物を出力
  std::sort(changed_pair.begin(), changed_pair.end());
  if( std::abs(max_front) > std::abs(min_front) ){
    max_front = std::abs(max_front);
  }else{
    max_front = std::abs(min_front);
  }
  min_front = -1.0*max_front;
  insert_box = Comp_width(changed_pair, max_front, min_front);//横方向について処理
  Get_box.insert(Get_box.end(), insert_box.begin(), insert_box.end());
  degree_value = -0.1f;
  for(int i = 0; i < Get_box.size(); i++){
    float com_dist = Degree_dist(Get_box[i]);
    if(com_dist > degree_value){//同じ値を得たとしたら,早い方をつかう.ただただだるいからであって,言うたら手抜き
      degree_value = com_dist;
      root = Get_box[i];
      get_flag = true;
    }
  }
  return get_flag;
}

// widthを計算するやつ....
std::vector< WPairFloat > scan_class::Comp_width(const std::vector<std::pair<float, float> >& pair_data, float& max, float& min)
{
  std::vector< WPairFloat >  return_box;
  return_box.resize(1);
  WPairFloat Wpair;
  int mode = -1;
  float big_width = -1.0f;
  float com_width = 0.0f;
  for(auto pair_itr = pair_data.begin(); pair_itr != pair_data.end(); pair_itr++){
    if(pair_itr != pair_data.begin() && pair_itr != (pair_data.end() - 1)){
      com_width = pair_itr->first - (pair_itr - 1)->first;
      Wpair = std::make_pair(*pair_itr, *(pair_itr - 1));
    }else if(pair_itr == pair_data.begin()){
      com_width = pair_itr->first - min;
      Wpair = std::make_pair(*pair_itr, std::make_pair(min , 0.0f));
    }else{
      com_width = max - pair_itr->first;
      Wpair = std::make_pair(std::make_pair(max , 0.0f), *pair_itr);
    }
    if(big_width < com_width){
      return_box.resize(1);
      big_width = com_width;
      return_box[0] = Wpair;
    }else if(big_width == com_width){
      return_box.push_back(Wpair);
    }
  }
  return return_box;
}



//2点の角度差を出す
inline float scan_class::Degree_dist(const WPairFloat& Wtarget)
{
  float first_p = Wtarget.first.first / pair_hypot(Wtarget.first);
  float second_p = Wtarget.second.first / pair_hypot(Wtarget.second);
  return std::abs(std::asin(first_p) - std::asin(second_p))*180.0f/M_PI;
}

inline float scan_class::pair_hypot(const std::pair<float, float>& pair_f)
{
  return hypot(pair_f.first - 0.0f, pair_f.second - 0.0f);
}

//Decisionsを変更する唯一の関数。。。ここ以外では変更しない
inline void scan_class::Set_Decisions(const Decisions& now_value)
{
  now_Decision_ = now_value;
}
//callback関数
void scan_class::callback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  if(Callback_flag_){
      std::cout << "leave....." << std::endl;
    return;
  }
  if(!Callback_flag_ && !Check_flag_){
    Check_flag_ = true;
      std::cout << "now" << std::endl;
    return;
  }
  if(Check_flag_ == true){
    Check_flag_ = false;
    Callback_flag_ = true;
    std::cout << "get" << std::endl;
    scan_data_ = *data;
  }
}
