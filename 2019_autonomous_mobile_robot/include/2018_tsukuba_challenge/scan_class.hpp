#ifndef SCAN_CLASS_NAME
#define SCAN_CLASS_NAME

#include <Mlib/libfm.hpp>
#include <Mlib/sub_core_LS.hpp>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef std::shared_ptr< message_filters::Subscriber<sensor_msgs::LaserScan> > SI_Ptr;
typedef std::pair< std::pair<float, float> , std::pair<float, float> > WPairFloat;
typedef enum MOVE_DECIDE {GO_FRONT, GO_BACK, STAY_NOW} MOVE_DECIDE;

struct Decisions
{
  MOVE_DECIDE State;
  std::pair<float, float> Root;
};


class scan_class : public sub_core_LS
{
public:
  scan_class(const std::string& topic, int Q_value, const std::string& name = "");
  ~scan_class();

  void scan_Situation();
  Decisions Out_Decisions() const;

private:
  void processing(const sensor_msgs::LaserScan& scan_msg);
  void Situation_processing(const pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud);
  void calc_processing_forroot(std::vector<std::pair<float, float> >& pair_front, std::vector<std::pair<float, float> >& pair_back);
  bool Get_root(WPairFloat& root, float& degree_value, const std::vector<std::pair<float, float> >& pair_data, const float& Sign_max, const float& Sign_min);
  std::vector< WPairFloat > Comp_width(const std::vector<std::pair<float, float> >& pair_data, float& max, float& min);
  inline std::pair<float, float> calc_root(const WPairFloat& root);
  inline float make_Spoint(const float& point1, const float& point2);
  inline float Degree_dist(const WPairFloat& Wtarget);
  inline float pair_hypot(const std::pair<float, float>& pair_f);
  inline void Set_Decisions(const Decisions& now_value);


  template <class S,class M> inline std::pair<M, S> pair_changer(const std::pair<S, M>& pair) const{
    return std::make_pair(pair.second, pair.first);
  }

  template <class S,class M> inline std::vector< std::pair<M, S> > pair_changer(const std::vector< std::pair<M, S> >& pair_box) const{
    std::vector< std::pair<M,S> > return_box;
    for(int i = 0; i < pair_box.size(); i++){
      return_box.push_back(std::make_pair(pair_box[i].second, pair_box[i].first));
    }
    return return_box;
  }



  void callback(const sensor_msgs::LaserScan::ConstPtr& data);





  ros::NodeHandle nh_;
  SI_Ptr sub_;

  sensor_msgs::LaserScan scan_data_;


  bool Callback_flag_, Check_flag_;
  float Danger_range_, Thre_deg_, Sign_min_, Sign_max_;

  Decisions now_Decision_;
};

#endif
