// メンバ変数には_を最後につけること

#ifndef JOY_CON_TYPE3
#define JOY_CON_TYPE3

// standard
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <utility>
// self-made
#include <joy_con_core/joy_con_core.hpp>
#include <joy_con_core/joy_con_param.hpp>
#include <helper_node/helper_pub.hpp>


namespace joy_con_type3{

  class JoyCon : public joy_con_core::JoyConCore
  {
  public:
    /*
    * @brief Constructor for JoyCon
     */
    JoyCon(void);
    /*
    * @brief Constructor for JoyCon
    * @param name : using for node name
     */
    JoyCon(std::string name);

    /*
    * @brief initializer for JoyConCore
    * @param name : using for node name
     */
    void initialize(std::string name);
    /*
    * @brief set current JoyData
    * @param axes_data : current JoyData's axes
    * @param buttons_data : current JoyData's buttons
     */
    void set_joy_data(std::vector<float> axes_data, std::vector<int> buttons_data);
    /*
    * @brief Check whether the current mode is on or off. Returns true if the mode is on.
     */
    bool check_onoff(bool update_flag);
    /*
    * @brief Shut down the process once
     */
    void shutdown_once(void);
    /*
    * @brief change data received from joy_node to control value according to class definition
    * @param joy_data : data received from joy_node
     */
    ambi_core::TwistParam get_converted_control_value(void);
    /*
    * @brief Destructor for ModeJoyCon
     */
    ~JoyCon(void);
  private:
    /*
    * @brief define how the augular velocity is determined
    * @param y_left_posi : Turn-Y's value
    * @param x_top_posi : Turn-X's value
    * @param back_flag : change any values for back mode, if it true.
     */
    float control_AngularVelocity_acceleration(float y_left_posi, float x_top_posi, bool indoor_flag, bool back_flag=false);
    /*
    * @brief define how the linear velocity is determined in Straight mode
    * @param accelerator : Accelerator's value.
    * @param back_flag : change any values for back mode, if it true.
     */
    float control_LinearVelocity_acceleration( bool indoor_flag, bool back_flag=false);
    /*
    * @brief define how the linear velocity is determined in Brake mode
     */
    float control_LinearVelocity_brake(bool outdoor_flag);
    /*
    * @brief Pre-calculation
     */
    void pre_calculation_LinearVelocity(void);

    // Must initialize
    bool initialize_flag_, shutdown_flag_;
    bool home_flag_, start_flag_, outdoor_flag_;
    int accelerate_counter_, outdoor_mode_;
    std::string current_mode_;
    // Mustn't initialize
    float constant_angular_, outdoor_constant_angular_, indoor_constant_angular_;
    joy_con_core::JoyConParam joy_con_param_;
    std::map<int, float> outdoor_velocity_map_, indoor_velocity_map_;
    helper_node::HelperPub pub_action_;
  };

};

#endif
