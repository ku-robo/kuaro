// メンバ変数には_を最後につけること

#ifndef JOY_CON_CORE
#define JOY_CON_CORE

// standard
#include <stdio.h>
#include <vector>
// self made
#include <ambi_core/core_parts.hpp>


namespace joy_con_core{
  class JoyConCore{
  public:
    /*
    * @brief initializer for JoyConCore
    * @param name : using for node name
     */
    virtual void initialize(std::string name)=0;
    /*
    * @brief set current JoyData
    * @param axes_data : current JoyData's axes
    * @param buttons_data : current JoyData's buttons
     */
    virtual void set_joy_data(std::vector<float> axes_data, std::vector<int> buttons_data)=0;
    /*
    * @brief Check whether the current mode is on or off. Returns true if the mode is on.
     */
    virtual bool check_onoff(bool update_flag)=0;
    /*
    * @brief Shut down the process once
     */
    virtual void shutdown_once(void)=0;
    /*
    * @brief change data received from joy_node to control value according to class definition
    * @param joy_data : data received from joy_node
     */
    virtual ambi_core::TwistParam get_converted_control_value(void)=0;
    /*
    * @brief Destructor for JoyConCore
     */
    virtual ~JoyConCore(void){}
  protected:
    /*
    * @brief Constructor for JoyConCore
     */
    JoyConCore(void){}
  };
};

#endif
