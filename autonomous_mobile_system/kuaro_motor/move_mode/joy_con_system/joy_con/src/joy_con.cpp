
#include <pluginlib/class_list_macros.h>
#include <joy_con/joy_con.hpp>

//register this mode as a MoveMode plugin
PLUGINLIB_EXPORT_CLASS(joy_con::ModeJoyCon, move_core::MoveMode)


namespace joy_con{
  ModeJoyCon::ModeJoyCon():joy_con_loader_("joy_con_core", "joy_con_core::JoyConCore"),initialize_flag_(false),update_flag_(false){}

  ModeJoyCon::ModeJoyCon(std::string name):
  joy_con_loader_("joy_con_core", "joy_con_core::JoyConCore"),
  initialize_flag_(false),update_flag_(false)
  {
    ROS_INFO(" [MoveModePlugin/ModeJoyCon] ModeJoyCon's second Constructor");
    initialize(name);
  }

  ModeJoyCon::~ModeJoyCon()
  {
    joy_con_type_.reset();
  }


  void ModeJoyCon::initialize(std::string name)
  {
    if(!initialize_flag_){
      ROS_INFO(" [MoveModePlugin/ModeJoyCon] Initialize %s ", name.c_str());
      ros::NodeHandle private_nh("~/"+name);
      std::string sub_topic_name, joy_con_type_name;
      // params
      private_nh.param("sub_topic_name", sub_topic_name, std::string("/joy"));
      private_nh.param("joy_con_type_name", joy_con_type_name, std::string("joy_con_type1/JoyCon"));

      sub_ = private_nh.subscribe(sub_topic_name, 1, &ModeJoyCon::Callback, this);
      //create a move safety system
      try {
        joy_con_type_ = joy_con_loader_.createInstance(joy_con_type_name);
        ROS_INFO(" [MoveModePlugin/ModeJoyCon] Load controll type %s", joy_con_type_name.c_str());
        joy_con_type_->initialize(joy_con_loader_.getName(joy_con_type_name));
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s safer, are you sure it is properly registered and that the containing library is built? Exception: %s", joy_con_type_name.c_str(), ex.what());
        exit(1);
      }
      initialize_flag_ = true;
    }else{
      ROS_WARN(" [MoveModePlugin/ModeJoyCon] This planner has already been initialized... doing nothing");
    }
  }

  ambi_core::TwistParam ModeJoyCon::get_command_value(void)
  {
    // return control value set by the control
    ROS_INFO(" [MoveModePlugin/ModeJoyCon] get_command_value of ModeJoyCon has been called");
    return joy_con_type_->get_converted_control_value();
  }

  void ModeJoyCon::use_encode_info(const ambi_core::PoseParam& pose_info, const ambi_core::TwistParam& twist_info)
  {
    ROS_INFO(" [MoveModePlugin/ModeJoyCon] This class doesn't use encoder information.");
  }

  bool ModeJoyCon::check_update_flag(bool change_flag)
  {
    if(!change_flag){
      // check the current mode
      ROS_INFO(" [MoveModePlugin/ModeJoyCon] Check update.");
      bool flag;
      flag = joy_con_type_->check_onoff(update_flag_);
      update_flag_ = false;
      return flag;
    }else{
      ROS_INFO(" [MoveModePlugin/ModeJoyCon] Down now");
      joy_con_type_->shutdown_once();
      return false;
    }
  }

  void ModeJoyCon::Callback(const sensor_msgs::Joy::ConstPtr& joy_data)
  {
    // set the current joy-con information on controller
    ROS_INFO(" [MoveModePlugin/ModeJoyCon] Callback update.");
    update_flag_ = true;
    joy_con_type_->set_joy_data(std::vector<float>(joy_data->axes), std::vector<int>(joy_data->buttons));
  }
};
