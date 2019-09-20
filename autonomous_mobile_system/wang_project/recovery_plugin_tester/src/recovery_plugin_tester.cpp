#include <recovery_plugin_tester/recovery_plugin_tester.hpp>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(wang_project, RecoveryPluginTester, wang_project::RecoveryPluginTester,  nav_core::RecoveryBehavior)

namespace wang_project
{
  RecoveryPluginTester::RecoveryPluginTester():
  global_costmap_(NULL), local_costmap_(NULL),tf_(NULL),
  initialize_flag_(false)
  {}

  RecoveryPluginTester::~RecoveryPluginTester()
  {
    delete world_model_;
  }

  void RecoveryPluginTester::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap,costmap_2d::Costmap2DROS* local_costmap)
  {
    if(!initialize_flag_)
    {
      name_ = name;
      global_costmap_ = global_costmap;
      local_costmap_ = local_costmap;
      tf_  = tf;
      //get some parameters from the parameter server
      ros::NodeHandle private_nh("~/" + name_);
      ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

      //we'll simulate every degree by default
      private_nh.param("sim_granularity", sim_granularity_, 0.017);
      private_nh.param("frequency", frequency_, 20.0);

      blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
      blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
      blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
      blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

      world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

      initialize_flag_ = true;
    }
    else
    {
      ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
  }

  void RecoveryPluginTester::runBehavior(void)
  {
    if(!initialize_flag_){
      ROS_ERROR("This object must be initialized before runBehavior is called");
      return;
    }

    if(global_costmap_ == NULL || local_costmap_ == NULL){
      ROS_ERROR("The costmaps passed to the RotateRecovery object cannot be NULL. Doing nothing.");
      return;
    }
  }
};
