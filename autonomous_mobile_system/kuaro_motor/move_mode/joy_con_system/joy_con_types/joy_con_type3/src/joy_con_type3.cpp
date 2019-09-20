#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <joy_con_type3/joy_con_type3.hpp>

//register this as a JoyConCore plugin
PLUGINLIB_EXPORT_CLASS(joy_con_type3::JoyCon, joy_con_core::JoyConCore)

namespace joy_con_type3{
  JoyCon::JoyCon():initialize_flag_(false),shutdown_flag_(false),home_flag_(true),start_flag_(false),outdoor_flag_(false),accelerate_counter_(0), outdoor_mode_(0), current_mode_(std::string("IndoorMode")){}

  JoyCon::JoyCon(std::string name):
  initialize_flag_(false),shutdown_flag_(false),home_flag_(true),start_flag_(false),outdoor_flag_(false),accelerate_counter_(0), outdoor_mode_(0), current_mode_(std::string("IndoorMode"))
  {
    ROS_INFO(" [JoyConCorePlugin/JoyCon] JoyCon's second Constructor");
    initialize(name);
  }

  JoyCon::~JoyCon(){}

  void JoyCon::initialize(std::string name)
  {
    if(!initialize_flag_){
      ROS_INFO(" [JoyConCorePlugin/JoyCon] Initialize %s ", name.c_str());
      // set axes and buttons param according to the joy-con used
      std::vector<float> init_axes{0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};
      std::vector<int> init_buttons(11, 0);

      std::vector<std::string> axes_params{"LS_Y", "LS_X", "L2", "RS_Y", "RS_X", "R2", "CK_Y", "CK_X"};
      std::vector<std::string> buttons_params{"Round_B", "Round_MR", "Round_ML", "Round_T", "L1", "R1", "BACK", "START", "HOME", "LSP", "RSP"};
      // Details...↓
      // std::vector<std::string> axes_params{"Left StiCK_Y", "Left Stick_X", "L2 Button", "Right Stick_Y", "Right Stick_X", "R2 Button", "The Cross Key_Y", "The Cross Key_X"};
      // std::vector<std::string> buttons_params{"Round Button_Bottom", "Round Button_MiddleRight", "Round Button_MiddleLeft", "Round Button_Top", "L1 Button", "R1 Button", "BACK Button", "START Button", "HOME Button", "Left Stick_Push", "Right Stick_Push"};

      // initialize joy_con_core::JoyConParam
      joy_con_param_.initialize(axes_params, buttons_params, init_axes, init_buttons);

      // initialize helper_node::HelperPub
      pub_action_.initialize("/kuaro_motor/joy_con", "action", "Action");

      // load and set ros param
      std::vector<std::pair<std::string,float> > param_float;
      std::vector<std::pair<std::string,std::string> > param_str;
      // define param

      param_float.push_back(std::make_pair("IndoorMaxLinearSpeed", 0.7f));//[m/s] about 2.5 km/h
      param_float.push_back(std::make_pair("IndoorMaxAngularSpeed", 2.0f));//[rad/s] about 112 degree/s
      param_float.push_back(std::make_pair("MaxLinearSpeed", 1.1f));//[m/s] about 4 km/h
      param_float.push_back(std::make_pair("MaxAngularSpeed", 0.8f));//[rad/s] about 45 degree/s
      param_float.push_back(std::make_pair("SafetyRatio", 1.2f));//[ - ]
      param_float.push_back(std::make_pair("BoostRatio", 1.5f));//[ - ]
      param_float.push_back(std::make_pair("TimeConstant", 1.0f));//[ s ]
      // about axes(a_act)
      param_str.push_back(std::make_pair("Accelerator", "R2"));
      param_str.push_back(std::make_pair("Brake", "L2"));
      param_str.push_back(std::make_pair("Turn_Y", "LS_Y"));
      param_str.push_back(std::make_pair("Turn_X", "LS_X"));
      param_str.push_back(std::make_pair("CS_Y", "CK_Y"));//CS = Constant Speed
      param_str.push_back(std::make_pair("CS_X", "CK_X"));
      // about buttons(b_act)
      param_str.push_back(std::make_pair("Straight", "Round_T"));
      param_str.push_back(std::make_pair("Turn_R", "Round_MR"));
      param_str.push_back(std::make_pair("Turn_L", "Round_ML"));
      param_str.push_back(std::make_pair("Back", "Round_B"));
      param_str.push_back(std::make_pair("OutdoorMode", "LSP"));
      param_str.push_back(std::make_pair("Boost", "RSP"));
      param_str.push_back(std::make_pair("Exp", "HOME"));
      param_str.push_back(std::make_pair("START", "START"));
      param_str.push_back(std::make_pair("FINISH", "BACK"));


      // set params
      joy_con_param_.set_param(name, param_float, param_str);

      // set the correspondence between axes,buttons and action
      std::vector<std::pair<std::string,std::string> > correspondence;
      // define. pair's first is action , second is joy_con
      correspondence = param_str;
      // set actions
      joy_con_param_.set_action(correspondence);

      // display the Configuration
      joy_con_param_.explain_joy_con();
      joy_con_param_.explain_method_of_operation();
      pre_calculation_LinearVelocity();
      constant_angular_ = indoor_constant_angular_;
      initialize_flag_ = true;
    }else{
      ROS_WARN(" [JoyConCorePlugin/JoyCon] This planner has already been initialized... doing nothing");
    }
  }

  void JoyCon::set_joy_data(std::vector<float> axes_data, std::vector<int> buttons_data)
  {
    // set the current joy_data
    joy_con_param_.set(axes_data, buttons_data);
  }

  bool JoyCon::check_onoff(bool update_flag)
  {
    ROS_INFO(" [JoyConCorePlugin/JoyCon] FUNC check_onoff");
    if(start_flag_){
      if(update_flag){
        if( joy_con_param_.b_act("FINISH") == 1){
          // check whether back buttons was pushed.
          ROS_INFO(" [JoyConCorePlugin/JoyCon] Current Mode is FINISH..... Finish Operation by Joycon");
          start_flag_ = false;
        }
      }else{
        ROS_INFO(" [JoyConCorePlugin/JoyCon] Current Mode is MOVING.....");
        joy_con_param_.set_initvalue();
      }
    }else{
      if(update_flag){
        if( joy_con_param_.b_act("START") == 1 ){
          // check whether start buttons was pushed.
          ROS_INFO(" [JoyConCorePlugin/JoyCon] Current Mode is START..... Start Operation by Joycon");
          start_flag_ = true;
        }
      }else{
        ROS_INFO(" [JoyConCorePlugin/JoyCon] Current Mode is WAITING.....");
        joy_con_param_.set_initvalue();
      }
    }
    return start_flag_;
  }

  void JoyCon::shutdown_once(void)
  {
    accelerate_counter_ = 0;
    // reset the current joy_data
    joy_con_param_.clear();
    shutdown_flag_ = true;
  }

  ambi_core::TwistParam JoyCon::get_converted_control_value(void)
  {
    ambi_core::TwistParam return_twist(0.0);
    if(!joy_con_param_.check_set()){
      accelerate_counter_ = 0;
      return return_twist;
    }
    if(shutdown_flag_){
      shutdown_flag_ = false;
      accelerate_counter_ = 0;
      ROS_INFO(" [JoyConCorePlugin/JoyCon] First Action after Shutdown once");
      return return_twist;
    }
    std::string current_mode="No-Data";
    std::vector<std::pair<std::string,std::string> > pub_data;
    // check whether home buttons was pushed.
    if( joy_con_param_.b_act("Exp") == 1 && home_flag_){
      home_flag_ = false;
      ROS_INFO(" [JoyConCorePlugin/JoyCon] Displays the operation method");
      joy_con_param_.explain_method_of_operation();
      return return_twist;
    }else if( joy_con_param_.b_act("Exp") == 0 && !home_flag_){
      home_flag_ = true;
    }

    float boost_value = 1.0f;
    if( outdoor_mode_ - joy_con_param_.b_act("OutdoorMode") < 0){
      if(outdoor_flag_ ){
        outdoor_flag_ = false;
        current_mode_ = "IndoorMode";
        constant_angular_ = indoor_constant_angular_;
      }else{
        outdoor_flag_ = true;
        current_mode_ = "OutdoorMode";
        constant_angular_ = outdoor_constant_angular_;
      }
    }
    outdoor_mode_ = joy_con_param_.b_act("OutdoorMode");

    // I wrote as clearly as possible...
    if(joy_con_param_.a_act("Brake") < 0.0 ){
      // in the case of pushing Brake
      // Brake.
      ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Brake", current_mode_.c_str());
      current_mode = "Brake";
      accelerate_counter_ = 0;
      return_twist.linear_x = 0.0;
      return_twist.angular_z = 0.0;
    }else if( 0.0 <= joy_con_param_.a_act("Brake") ){
      // in the case of not pushing Brake
      if( joy_con_param_.a_act("Accelerator") < 0.0 ){
        // in the case of pushing Accelerator

        // Round_Buttons and Left_Stick
        if(joy_con_param_.b_act("Straight") + joy_con_param_.b_act("Back") > 0){
          // Only Go straight or back mode
          if( joy_con_param_.b_act("Back") == 0){
            // Straight ahead only
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Straight ahead only", current_mode_.c_str());
            current_mode = "Straight ahead only";
            return_twist.linear_x = control_LinearVelocity_acceleration(outdoor_flag_);
            return_twist.angular_z = 0.0;
          }else if( joy_con_param_.b_act("Straight") == 0 ){
            // Only Go back
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Back mode", current_mode_.c_str());
            current_mode = "Back mode";
            return_twist.linear_x = control_LinearVelocity_acceleration(outdoor_flag_, true);
            return_twist.angular_z = control_AngularVelocity_acceleration(joy_con_param_.a_act("Turn_Y"), joy_con_param_.a_act("Turn_X"), outdoor_flag_, true);
          }
        }else if(joy_con_param_.a_act("CS_Y") != 0.0 || joy_con_param_.a_act("CS_X") != 0.0 ){
          // Go straight and turn
          // straight
          ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Cross Stick driving",current_mode_.c_str());
          current_mode = "Cross Stick driving";
          // about Linear
          if(joy_con_param_.a_act("CS_X") < 0.0){
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Cross Stick driving Back ",current_mode_.c_str());
            return_twist.linear_x = control_LinearVelocity_acceleration(outdoor_flag_, true);
          }else{
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Cross Stick driving Straight ",current_mode_.c_str());
            return_twist.linear_x = control_LinearVelocity_acceleration(outdoor_flag_);
          }
          if( 0.0 <  joy_con_param_.a_act("CS_X")){
            boost_value = boost_value*joy_con_param_.param("BoostRatio");
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Speed Boost %lf",current_mode_.c_str(), boost_value);
            return_twist.linear_x = boost_value*return_twist.linear_x;
          }

          // about Angular
          return_twist.angular_z = 0.0f;
          if( 0.0 < joy_con_param_.a_act("CS_Y") ){
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Cross Stick driving Turn Left",current_mode_.c_str());
            return_twist.angular_z = constant_angular_;
          }else if( joy_con_param_.a_act("CS_Y") < 0.0 ){
            ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Cross Stick driving Turn Right",current_mode_.c_str());
            return_twist.angular_z = -constant_angular_;
          }
        }else{
          // Go straight and turn
          // straight
          ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Normal driving",current_mode_.c_str());
          current_mode = "Normal driving";
          return_twist.linear_x = control_LinearVelocity_acceleration(outdoor_flag_);
          return_twist.angular_z = control_AngularVelocity_acceleration(joy_con_param_.a_act("Turn_Y"), joy_con_param_.a_act("Turn_X"), outdoor_flag_);
        }
        // Boost
        if(joy_con_param_.b_act("Boost") > 0){
          boost_value = boost_value*joy_con_param_.param("BoostRatio");
          ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Speed Boost %lf",current_mode_.c_str() , boost_value);
          pub_data.push_back(std::make_pair("Speed Boost", std::to_string(joy_con_param_.param("BoostRatio"))));
          pub_action_.set(current_mode, pub_data, helper_node::NOTHING);
          pub_data.pop_back();
          return_twist.linear_x = boost_value*return_twist.linear_x;
        }
      }else if( 0.0 <= joy_con_param_.a_act("Accelerator")){
        // reset counter of accelerator to 0 because a_act[Accelerator] was not pushed.
        if( accelerate_counter_ == 0){
          // in the case of not pushing Accelerator
          if(joy_con_param_.b_act("Turn_R") + joy_con_param_.b_act("Turn_L") > 0){
            return_twist.linear_x = 0.0;
            if( joy_con_param_.b_act("Turn_L") == 0){
              // turn Right
              ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Turn Right only",current_mode_.c_str() );
              current_mode = "Turn Right only";
              return_twist.angular_z = -constant_angular_;
            }else if( joy_con_param_.b_act("Turn_R") == 0 ){
              // turn Left
              ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : Turn Left only",current_mode_.c_str() );
              current_mode = "Turn Left only";
              return_twist.angular_z = constant_angular_;
            }
          }
        }else{
          // Deceleration mode
          ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Action : NO Accelerator mode",current_mode_.c_str() );
          current_mode = "NO Accelerator mode";
          accelerate_counter_ = 0;
          return_twist.linear_x = 0.0;
          return_twist.angular_z = 0.0;
        }
      }
    }
    ROS_INFO(" [JoyConCorePlugin/JoyCon] %s-Value : Linear_X = %lf [m/s], Angular_Z = %lf [rad/s]",current_mode_.c_str() , return_twist.linear_x, return_twist.angular_z);
    pub_data.push_back(std::make_pair("Linear_X", std::to_string(return_twist.linear_x)));
    pub_data.push_back(std::make_pair("Angular_Z", std::to_string(return_twist.angular_z)));
    pub_action_.set(current_mode, pub_data);
    pub_action_.publish(helper_node::FULL_CLEAR);
    // reset the current joy_data
    joy_con_param_.clear();
    return return_twist;
  }

  float JoyCon::control_AngularVelocity_acceleration(float y_left_posi, float x_top_posi, bool outdoor_flag, bool back_flag)
  {
    // Change the setting so that the right side of the X axis is positive and the upper part of the Y axis is positive.
    float x_val = -y_left_posi;
    float y_val = x_top_posi;
    float r_val =  std::hypot(y_left_posi, x_top_posi);
    static const float outdoor_max_angular = joy_con_param_.param("MaxAngularSpeed");
    static const float indoor_max_angular = joy_con_param_.param("IndoorMaxAngularSpeed");
    static const float dead_area = 0.20f;
    static const float max_y = 0.99f;// about 85 degree
    static const float first_y = 0.78f;
    static const float second_y = 0.35f;
    static const float third_y = -0.45f;
    static const float min_y = -0.95f;

    if(back_flag) y_val = -y_val;
    // check whether it is on the dead area.

    if( dead_area < r_val && 0.00 < (max_y - y_val)*(y_val - min_y) ){

      float return_angular = indoor_max_angular;//the return value
      if(outdoor_flag) return_angular = outdoor_max_angular;

      //apply ratio by degree to return_angular
      if( first_y < y_val ){
        return_angular = return_angular*0.25;
      }else if( second_y < y_val ){
        return_angular = return_angular*0.50;
      }else if( third_y < y_val ){
        return_angular = return_angular*0.75;
      }

      // express direction
      if( x_val > 0.00){
        return_angular = -return_angular;
      }
      return return_angular;
    }else{
      // it is on the dead area. so, return 0
      return 0.00f;
    }
  }

  float JoyCon::control_LinearVelocity_acceleration(bool outdoor_flag, bool back_flag)
  {
    static const int timeconstant = joy_con_param_.param("TimeConstant");
    static const int max_count = 5*timeconstant;
    // if current mode is Straight.
    if(!back_flag){
      if( max_count <= accelerate_counter_ ){
        accelerate_counter_ = max_count;
      }else{
        accelerate_counter_++;
      }
    }else{// if current mode is Back.
      if( accelerate_counter_ <= -max_count ){
        accelerate_counter_ = -max_count;
      }else{
        accelerate_counter_--;
      }
    }
    if(outdoor_flag)  return outdoor_velocity_map_[accelerate_counter_];
    return indoor_velocity_map_[accelerate_counter_];
  }

  float JoyCon::control_LinearVelocity_brake(bool outdoor_flag)
  {
    if( accelerate_counter_ < 0 ){
      accelerate_counter_++;
    }else if( 0 < accelerate_counter_ ){
      accelerate_counter_--;
    }
    if(outdoor_flag) return outdoor_velocity_map_[accelerate_counter_];
    return indoor_velocity_map_[accelerate_counter_];
  }

  void JoyCon::pre_calculation_LinearVelocity(void)
  {
    int timeconstant = joy_con_param_.param("TimeConstant");
    int max_count = 5*timeconstant;
    float outdoor_Max_V = joy_con_param_.param("MaxLinearSpeed")/joy_con_param_.param("SafetyRatio");// the set max speed
    float indoor_Max_V = joy_con_param_.param("IndoorMaxLinearSpeed")/joy_con_param_.param("SafetyRatio");// the set max speed


    // set speed 'zero' to 0 on the map.
    outdoor_velocity_map_[0] = 0.0;
    indoor_velocity_map_[0] = 0.0;
    // set speed to i on the map.
    for(int i = 1; i <= max_count; i++){
      float Velocity = outdoor_Max_V*( 1 - std::exp(-i/timeconstant) );
      outdoor_velocity_map_[i] = Velocity;
      outdoor_velocity_map_[-i] = -Velocity;

      Velocity = indoor_Max_V*( 1 - std::exp(-i/timeconstant) );
      indoor_velocity_map_[i] = Velocity;
      indoor_velocity_map_[-i] = -Velocity;
    }
    outdoor_constant_angular_ = joy_con_param_.param("MaxAngularSpeed")/joy_con_param_.param("SafetyRatio");
    indoor_constant_angular_ =  joy_con_param_.param("IndoorMaxAngularSpeed")/joy_con_param_.param("SafetyRatio");
  }
};
