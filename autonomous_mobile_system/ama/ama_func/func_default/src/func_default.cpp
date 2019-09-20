
#include <pluginlib/class_list_macros.h>
#include <func_default/func_default.hpp>

//register this mode as a AMAFunc plugin
PLUGINLIB_EXPORT_CLASS(func_default::FuncDefault, ama_core::AMAFunc)

namespace func_default{
  FuncDefault::FuncDefault(void):initialize_flag_(false),change_flag_(false),maintain_flag_(true){}

  FuncDefault::FuncDefault(std::string name, tf::TransformListener* tf):
  initialize_flag_(false),change_flag_(false),maintain_flag_(true)
  {
    initialize(name, tf);
  }

  FuncDefault::~FuncDefault(void){}

  void FuncDefault::initialize(std::string name, tf::TransformListener* tf)
  {
    if(!initialize_flag_){
      tf_ = tf;
      ros::NodeHandle private_nh("~/"+ name);
      std::string file_name;
      private_nh.param("finalwaypoint_path", file_name, std::string("/home/robo/ama_test/finalwaypoint.txt"));
      private_nh.param("move_method", move_method_, std::string("default"));



      // set data from file to waypoint_vec_
      FILE* fp;
      fp = fopen(file_name.c_str(), "r");
      if (fp == NULL)
      {
        ROS_ERROR("file path error!");
        return;
      }
      ama_struct::GetPoint tmp("/map");
      while(fscanf(fp,"%f,%f,%f,%f,%f,%f,%f\n", &tmp.position_x, &tmp.position_y, &tmp.position_z, &tmp.orientation_x, &tmp.orientation_y, &tmp.orientation_z, &tmp.orientation_w) != EOF)
      {
        waypoint_vec_.push_back(tmp);
      }
      fclose(fp);

      initialize_flag_ = true;
    }else{
      ROS_WARN(" [AMAFuncPlugin/FuncDefault][Func:initialize] This planner has already been initialized... doing nothing");
    }
  }

  bool FuncDefault::func_work(void)
  {
    if(!initialize_flag_){
      ROS_WARN(" [AMAFuncPlugin/FuncDefault][Func:func_work] This planner is not yet initialized... You need to initialize it.");
      return true;
    }
    ROS_INFO(" [AMAFuncPlugin/FuncDefault][Func:func_work] Moving Method is Default");

    return true;
  }

  bool FuncDefault::update_flag(void)
  {
    return change_flag_;
  }


  bool FuncDefault::maintain_flag(void)
  {
    return maintain_flag_;
  }

  std::vector<ama_struct::GetPoint> FuncDefault::GetPoints(void)
  {
    return waypoint_vec_;
  }

  std::string FuncDefault::get_move_method(void)
  {
    return move_method_;
  }

};
