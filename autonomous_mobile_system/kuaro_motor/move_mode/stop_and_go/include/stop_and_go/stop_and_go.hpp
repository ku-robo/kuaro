// メンバ変数には_を最後につけること

#ifndef STOP_AND_GO
#define STOP_AND_GO

// standard
#include <string>
// ROS
#include <ros/ros.h>
// ROSmsgs
#include <std_msgs/String.h>
// self made
#include <ambi_core/core_parts.hpp>
#include <move_core/move_mode.hpp>



namespace stop_and_go{

  class ModeStopAndGo : public move_core::MoveMode
  {
  public:
    /*
    * @brief Constructor for ModeStopAndGo
     */
    ModeStopAndGo(void);
    /*
    * @brief Constructor for ModeStopAndGo
    * @param name : using for node name
     */
    ModeStopAndGo(std::string name);
    /*
    * @brief initializer for ModeStopAndGo
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
    * @brief check update_flag_'s value.
     */
    bool check_update_flag(bool change_flag);
    /*
    * @brief Destructor for ModeStopAndGo
     */
    ~ModeStopAndGo(void);
  private:
    /*
    * @brief Define user's commands and set their to command_map_. If you want to add the command you need, you must write the command on the first line of this function.
     */
    void set_command(void);
    /*
    * @brief Callback function for ModeStopAndGo
    * @param string_data : This is the command information received from helper
     */
    void Callback(const std_msgs::String::ConstPtr& string_data);

    // Must initialize
    bool initialize_flag_;
    std::string sub_data_;

    // ros
    ros::Subscriber sub_;

    // Mustn't initialize
    std::map<std::string, int> command_map_;
  };
};

#endif
