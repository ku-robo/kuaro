// メンバ変数には_を最後につけること

#ifndef KUARO_MOVER
#define KUARO_MOVER

// 標準
#include <stdio.h>
#include <XmlRpcValue.h>
// Boost
#include <boost/shared_ptr.hpp>
// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
// ROSmsgs
#include <std_msgs/String.h>
// 自作
#include <ambi_core/core_parts.hpp>
#include <move_core/move_mode.hpp>
#include <move_core/move_safety.hpp>


static const double ENCODER_GET_TIME = 0.0333;//エンコーダーの値を取る時間幅[sec]

namespace kuaro_mover {

  class KuaroMover {
  public:
    KuaroMover(void);
    ~KuaroMover(void);
    /*
    * @brief Work as main
    */
    void run(void);
  private:
    /*
    * @brief load MoveCores and push back them to move_modes_
    * @param node: for getparam
    */
    bool loadMoveCores(ros::NodeHandle node);
    /*
    * @brief load default MoveCore
    */
    void loadDefaultMode(void);
    /*
    * @brief make move_mode_encoders_ by checking whether encode_flag_ is true.
    */
    void makeMoveMode_encoders(void);
    /*
    * @brief update flag to True, just before we'll subscribe any topics.
    */
    void updateflag__init(void);
    /*
    * @brief check whether update flag is True against move_modes. We'll use the data we can catch true at first. And if we could get new data, return true.
    */
    bool check_update_data(ambi_core::TwistParam& twist_data);
    /*
    * @brief for okatech. they are okatech commands.
    */
    void send_command(const ambi_core::TwistParam& sub);
    std::string serial_linear(const double& linear_vel);
    std::string serial_angular(const double&  angular_vel);
    void serial_Callback(const std_msgs::String::ConstPtr& serials);
    void cmd_timer(void);

    // ROS
    ros::Subscriber serial_sub_;
    ros::Publisher serial_pub_;
    ros::Timer timer_;

    // PoseParam---KUAROへの指示
    ambi_core::TwistParam sub_command_twist_;

    // OdomParam---エンコーダー情報
    ambi_core::PoseParam pub_odom_pose_;
    ambi_core::TwistParam pub_odom_twist_;

    // roslaunch-param
    std::string sub_serial_topic_, pub_serial_topic_;
    int loop_rate_;

    // 変数-初期化順
    pluginlib::ClassLoader<move_core::MoveMode> move_mode_loader_;// plugin
    pluginlib::ClassLoader<move_core::MoveSafety> move_safety_loader_;// plugin
    double th_rad_;//odomのためのパーツ
    bool updateflag_;// データ更新時のflag

    // 変数・初期化しない勢
    bool mode_move_base_append_, mode_wireless_append_;
    boost::shared_ptr<move_core::MoveSafety> safety_system_;
    std::vector< boost::shared_ptr<move_core::MoveMode> > move_modes_, move_mode_encoders_;
// template

  };

}

#endif
