#include <sstream>
#include <istream>

#include <pluginlib/class_list_macros.h>
#include <stop_and_go/stop_and_go.hpp>

//register this mode as a MoveMode plugin
PLUGINLIB_EXPORT_CLASS(stop_and_go::ModeStopAndGo, move_core::MoveMode)

namespace stop_and_go{
  ModeStopAndGo::ModeStopAndGo():
  initialize_flag_(false),sub_data_("go"){}

  ModeStopAndGo::ModeStopAndGo(std::string name):
  initialize_flag_(false),sub_data_("go")
  {
    ROS_INFO(" [MoveModePlugin/ModeStopAndGo] ModeStopAndGo's second Constructor");
    initialize(name);
  }

  ModeStopAndGo::~ModeStopAndGo(){}

  void ModeStopAndGo::set_command(void)
  {
    std::string command = "stop,go";// Write the necessary command. "," Must be written between each command.
    std::vector<std::string> command_vector;
    std::stringstream ss{command};
    std::string buf;
    while (std::getline(ss, buf, ',')){
      command_vector.push_back(buf);
    }
    for(unsigned int i=0; i<command_vector.size(); i++){
      command_map_[command_vector[i]] = i;
    }
  }

  void ModeStopAndGo::initialize(std::string name)
  {
    if(!initialize_flag_){
      ROS_INFO(" [MoveModePlugin/ModeStopAndGo] Initialize %s ", name.c_str());
      ros::NodeHandle private_nh("~/"+name);
      std::string sub_topic_name;
      // params
      private_nh.param("sub_topic_name", sub_topic_name, std::string("stop_and_go"));
      sub_ = private_nh.subscribe(sub_topic_name, 3, &ModeStopAndGo::Callback, this);
      initialize_flag_ = true;

      // define user's commands and set their to command_map_.
      set_command();
    }else{
      ROS_WARN(" [MoveModePlugin/ModeStopAndGo] This planner has already been initialized... doing nothing");
    }
  }

  ambi_core::TwistParam ModeStopAndGo::get_command_value(void)
  {
    ROS_INFO(" [MoveModePlugin/ModeStopAndGo] get_command_value of ModeStopAndGo has been called");
    ambi_core::TwistParam return_twist;
    return_twist.init(0.0);
    return return_twist;
  }

  void ModeStopAndGo::use_encode_info(const ambi_core::PoseParam& pose_info, const ambi_core::TwistParam& twist_info)
  {
    ROS_INFO(" [MoveModePlugin/ModeStopAndGo] This class doesn't use encoder information.");
  }

  bool ModeStopAndGo::check_update_flag(bool change_flag)
  {
    // The main task of this class is to stop moving and move again.
    // Therefore, check the received command and determine the value of update_flag accordingly.

    // Check whether command_map_ has current sub_data_.
    ROS_INFO(" [MoveModePlugin/ModeStopAndGo] Check update.");
    bool update_flag;
    if(command_map_.count(sub_data_)) {

      switch(command_map_[sub_data_]){
        // command is 'stop'
        case 0:
          ROS_INFO(" [MoveModePlugin/ModeStopAndGo] Run command 'stop'");
          update_flag = true;
          // So, a function 'get_command_value' will be called
          break;
        // command is 'go'
        case 1:
          ROS_INFO(" [MoveModePlugin/ModeStopAndGo] Run command 'go'");
          update_flag = false;
          // So, a function 'get_command_value' won't be called
          break;
      }
    }else{
      ROS_INFO(" [MoveModePlugin/ModeStopAndGo] This command '%s' is not set on command_map_. ", sub_data_.c_str());
      update_flag = false;
      // So, a function 'get_command_value' won't be called
    }
    return update_flag;
  }

  void ModeStopAndGo::Callback(const std_msgs::String::ConstPtr& string_data)
  {
    ROS_INFO(" [MoveModePlugin/ModeStopAndGo] Callback update.");
    sub_data_ = string_data->data;
  }
};
