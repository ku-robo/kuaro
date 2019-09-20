#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <joy_con_type1/joy_con_type1.hpp>

//register this as a JoyConCore plugin
PLUGINLIB_EXPORT_CLASS(joy_con_type1::JoyCon, joy_con_core::JoyConCore)


namespace joy_con_type1{
  JoyCon::JoyCon():initialize_flag_(false),home_flag_(true),start_flag_(false),accelerate_counter_(0){}

  JoyCon::JoyCon(std::string name):
  initialize_flag_(false),home_flag_(true),start_flag_(false),accelerate_counter_(0)
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
      param_str.push_back(std::make_pair("CS_X", "CS_X"));
      // about buttons(b_act)
      param_str.push_back(std::make_pair("Straight", "Round_T"));
      param_str.push_back(std::make_pair("Turn_R", "Round_MR"));
      param_str.push_back(std::make_pair("Turn_L", "Round_ML"));
      param_str.push_back(std::make_pair("Back", "Round_B"));
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
  }

  ambi_core::TwistParam JoyCon::get_converted_control_value(void)
  {
    ambi_core::TwistParam return_twist(0.0);
    if(!joy_con_param_.check_set()){
      accelerate_counter_ = 0;
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

    // I wrote as clearly as possible...
    if(joy_con_param_.a_act("Brake") < 0.0 ){
      // in the case of pushing Brake
      // Brake.
      ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Brake");
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
            ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Straight ahead only");
            current_mode = "Straight ahead only";
            return_twist.linear_x = control_LinearVelocity_acceleration();
            return_twist.angular_z = 0.0;
          }else if( joy_con_param_.b_act("Straight") == 0 ){
            // Only Go back
            ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Back mode");
            current_mode = "Back mode";
            return_twist.linear_x = control_LinearVelocity_acceleration(true);
            return_twist.angular_z = control_AngularVelocity_acceleration(joy_con_param_.a_act("Turn_Y"), joy_con_param_.a_act("Turn_X"), true);
          }
        }else{
          // Go straight and turn
          // straight
          ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Normal driving");
          current_mode = "Normal driving";
          return_twist.linear_x = control_LinearVelocity_acceleration();
          return_twist.angular_z = control_AngularVelocity_acceleration(joy_con_param_.a_act("Turn_Y"), joy_con_param_.a_act("Turn_X"));
        }
        // Boost
        if(joy_con_param_.b_act("Boost") > 0){
          ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Speed Boost %lf", joy_con_param_.param("BoostRatio"));
          pub_data.push_back(std::make_pair("Speed Boost", std::to_string(joy_con_param_.param("BoostRatio"))));
          pub_action_.set(current_mode, pub_data, helper_node::NOTHING);
          pub_data.pop_back();
          return_twist.linear_x = return_twist.linear_x * joy_con_param_.param("BoostRatio");
        }
      }else if( 0.0 <= joy_con_param_.a_act("Accelerator")){
        // reset counter of accelerator to 0 because a_act[Accelerator] was not pushed.
        if( accelerate_counter_ == 0){
          // in the case of not pushing Accelerator
          if(joy_con_param_.b_act("Turn_R") + joy_con_param_.b_act("Turn_L") > 0){
            return_twist.linear_x = 0.0;
            if( joy_con_param_.b_act("Turn_L") == 0){
              // turn Right
              ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Turn Right only");
              current_mode = "Turn Right only";
              return_twist.angular_z = -joy_con_param_.param("MaxAngularSpeed")/joy_con_param_.param("SafetyRatio")/2.0;
            }else if( joy_con_param_.b_act("Turn_R") == 0 ){
              // turn Left
              ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Turn Left only");
              current_mode = "Turn Left only";
              return_twist.angular_z = joy_con_param_.param("MaxAngularSpeed")/joy_con_param_.param("SafetyRatio")/2.0;
            }
          }
        }else{
          // Deceleration mode
          ROS_INFO(" [JoyConCorePlugin/JoyCon] Action : Deceleration mode");
          current_mode = "Deceleration mode";
          return_twist.linear_x = control_LinearVelocity_brake();
          return_twist.angular_z = 0.0;
        }
      }
    }
    ROS_INFO(" [JoyConCorePlugin/JoyCon] Value : Linear_X = %lf [m/s], Angular_Z = %lf [rad/s]", return_twist.linear_x, return_twist.angular_z);
    pub_data.push_back(std::make_pair("Linear_X", std::to_string(return_twist.linear_x)));
    pub_data.push_back(std::make_pair("Angular_Z", std::to_string(return_twist.angular_z)));
    pub_action_.set(current_mode, pub_data);
    pub_action_.publish(helper_node::FULL_CLEAR);
    // reset the current joy_data
    joy_con_param_.clear();
    return return_twist;
  }

  float JoyCon::control_AngularVelocity_acceleration(float y_left_posi, float x_top_posi, bool back_flag)
  {
    // Change the setting so that the right side of the X axis is positive and the upper part of the Y axis is positive.
    float x_val = -y_left_posi;
    float y_val = x_top_posi;
    float r_val =  std::hypot(y_left_posi, x_top_posi);
    static const float max_angular = joy_con_param_.param("MaxAngularSpeed");
    static const float dead_area = 0.20f;
    static const float half_area = 0.60f;
    static const float max_y = 0.99f;// about 85 degree
    static const float first_y = 0.35f;
    static const float second_y = 0.77f;

    if(back_flag) y_val = -y_val;
    // check whether it is on the dead area.

    if( dead_area < r_val && 0.00 < (max_y - y_val)*(y_val+0.05) ){

      float return_angular = max_angular;//the return value

      //apply ratio by degree to return_angular
      if( second_y < y_val ){
        return_angular = return_angular*2.0/7.0;
      }else if( first_y < y_val ){
        return_angular = return_angular*5.0/7.0;
      }

      // apply ratio by area to return_angular
      if(r_val < half_area){
        return_angular = return_angular * 0.50;
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

  float JoyCon::control_LinearVelocity_acceleration(bool back_flag)
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
    return velocity_map_[accelerate_counter_];;
  }

  float JoyCon::control_LinearVelocity_brake(void)
  {
    if( accelerate_counter_ < 0 ){
      accelerate_counter_++;
    }else if( 0 < accelerate_counter_ ){
      accelerate_counter_--;
    }
    return velocity_map_[accelerate_counter_];
  }

  void JoyCon::pre_calculation_LinearVelocity(void)
  {
    int timeconstant = joy_con_param_.param("TimeConstant");
    int max_count = 5*timeconstant;
    float Max_V = joy_con_param_.param("MaxLinearSpeed")/joy_con_param_.param("SafetyRatio");// the set max speed

    // set speed 'zero' to 0 on the map.
    velocity_map_[0] = 0.0;
    // set speed to i on the map.
    for(int i = 1; i <= max_count; i++){
      float Velocity = Max_V*( 1 - std::exp(-i/timeconstant) );
      velocity_map_[i] = Velocity;
      velocity_map_[-i] = -Velocity;
    }
  }
};
