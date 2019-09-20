// メンバ変数には_を最後につけること

#ifndef JOY_CON
#define JOY_CON

// standard
#include <string>
#include <vector>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
// ROSmsgs
#include <sensor_msgs/Joy.h>
// self made
#include <ambi_core/core_parts.hpp>
#include <move_core/move_mode.hpp>
#include <joy_con_core/joy_con_core.hpp>


namespace joy_con{

  class ModeJoyCon : public move_core::MoveMode
  {
  public:
    /*
    * @brief Constructor for ModeJoyCon
     */
    ModeJoyCon(void);
    /*
    * @brief Constructor for ModeJoyCon
    * @param name : using for node name
     */
    ModeJoyCon(std::string name);
    /*
    * @brief initializer for ModeJoyCon
    * @param name : using for node name
     */
    void initialize(std::string name);
    /*
    * @brief give you commands this class sets.
     */
    ambi_core::TwistParam get_command_value(void);
    /*
    * @brief give you commands this class sets.
     */
    void use_encode_info(const ambi_core::PoseParam& pose_info, const ambi_core::TwistParam& twist_info);
    /*
    * @brief check whether there was an update.
     */
    bool check_update_flag(bool change_flag);
    /*
    * @brief Destructor for ModeJoyCon
     */
    ~ModeJoyCon(void);
  private:
    /*
    * @brief Callback function for ModeJoyCon
    * @param joy_data : This is the command information received from joy
     */
    void Callback(const sensor_msgs::Joy::ConstPtr& joy_data);



    // Must initialize
    pluginlib::ClassLoader<joy_con_core::JoyConCore> joy_con_loader_;// plugin
    bool initialize_flag_, update_flag_;
    // Mustn't initialize
    boost::shared_ptr<joy_con_core::JoyConCore> joy_con_type_;
    // ros
    ros::Subscriber sub_;

    //
  };
};

#endif
