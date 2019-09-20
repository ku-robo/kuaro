// メンバ変数には_を最後につけること

#ifndef RECOVERY_PLUGIN_TESTER
#define RECOVERY_PLUGIN_TESTER

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

namespace wang_project{

  class RecoveryPluginTester : public nav_core::RecoveryBehavior
  {
  public:
    RecoveryPluginTester(void);

    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);


    void runBehavior(void);
    virtual ~RecoveryPluginTester(void);
    private:

    /*costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
    std::string name_;
    tf::TransformListener* tf_; */
    costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
    costmap_2d::Costmap2D costmap_;
    std::string name_;
    tf::TransformListener* tf_;
    //bool initialized_;
    bool initialize_flag_;
    double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
    base_local_planner::CostmapModel* world_model_;

  };

};

#endif
