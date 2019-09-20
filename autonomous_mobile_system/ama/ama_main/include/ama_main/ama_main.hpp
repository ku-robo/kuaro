// メンバ変数には_を最後につけること

#ifndef AMA_MAIN
#define AMA_MAIN

// 標準
#include <stdio.h>
#include <math.h>
#include <XmlRpcValue.h>
#include <utility>
#include <map>
// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
// ROS-msgs
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
// Self-made
#include <ama_struct/get_point.hpp>
#include <ama_core/ama_func.hpp>
#include <ama_core/ama_move.hpp>


namespace ama_main{



  class AMAMain{
  public:
    AMAMain(tf::TransformListener& tf);

    void run(void);

    virtual ~AMAMain(void);
  private:
    // Functions related to WayPointThread
    /*
    * @brief
    */
    void WayPointThread(void);
    /*
    * @brief
    */
    void proccesing2get_waypoint(ama_struct::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag, const std::string& using_func_name );
    /*
    * @brief
    */
    bool get_waypoint(ama_struct::GetPoint& get_wp, float& target_change_dist, const std::vector<ama_struct::GetPoint>& using_func_vec, unsigned int current_index, bool& last_flag);


    /*
    * @brief get original pose position on the destination_frame coorfinate
    */
    ama_struct::GetPoint get_origin_position(const ros::Time& original_time, const std::string& original_frame= "/base_link", const std::string& destination_frame = "/map");
    /*
    * @brief get original pose position on the destination_frame coorfinate
    */
    ama_struct::GetPoint get_origin_position_now(const std::string& original_frame = "/base_link", const std::string& destination_frame = "/map");



    /*
    * @brief load MoveCores and push back them to move_modes_
    * @param node: for getparam
    */
    bool loadAMAFuncs(ros::NodeHandle node, const visualization_msgs::Marker& points);
    /*
    * @brief load default MoveCore
    */
    void loadDefaultFunc(const visualization_msgs::Marker& points);
    /*
    * @brief load MoveCores and push back them to move_modes_
    * @param node: for getparam
    */
    bool loadAMAMoves(ros::NodeHandle node);
    /*
    * @brief load default MoveCore
    */
    void loadDefaultMove(void);

    // Main
    tf::TransformListener& tf_;
    const std::string default_str_, end_func_str_;// Must initialize
    pluginlib::ClassLoader<ama_core::AMAFunc> ama_funcs_loader_;// Must initialize/ plugin
    pluginlib::ClassLoader<ama_core::AMAMove> ama_moves_loader_;// Must initialize/ plugin
    ros::Publisher marker_pub_;
    visualization_msgs::Marker points_;
    std::map<std::string, visualization_msgs::Marker> eachfunc_points_;
    // ROSPARAM
    float dist_target_posi_, target_change_dist_, wp_angle_threshold_;
    // WayPointThread
    boost::thread* waypoint_thread_;
    // plugins
    std::map< std::string, boost::shared_ptr<ama_core::AMAFunc> > ama_funcs_map_;
    std::map< std::string, boost::shared_ptr<ama_core::AMAMove> > ama_moves_map_;
    // For WayPoint
    bool wp_nextfunc_flag_;
    std::vector< std::pair<int, std::string> > wp_nextfunc_vec_;
    ama_struct::GetPoint current_pose_;
    float wp_goal_judg_threshold_ratio_;// Must initialize
    unsigned int advance_count_;

    // Thread Shared variable
    // For WayPoint---AMAFunc
    boost::recursive_mutex waypoint_mutex_;
    bool waypoint_flag_;// Must initialize
    std::map< std::string, std::vector<ama_struct::GetPoint> > waypoint_map_;
    // For MoveGoal---AMAMove
    boost::recursive_mutex movegoal_mutex_;
    bool movegoal_flag_;// Must initialize
    ama_struct::GetPoint movegoal_next_g_;
    // For Function---Shared Func:run
    boost::recursive_mutex func_mutex_;
    bool func_flag_;// Must initialize
    std::string func_next_name_;// Must initialize
    // For Break---AMAMove
    boost::recursive_mutex break_mutex_;
    bool break_flag_;// Must initialize
    // For Aborted---AMAMove
    boost::recursive_mutex aborted_mutex_;
    bool aborted_flag_;// Must initialize
    // For External instructions---AMAMove
    boost::recursive_mutex exinstruct_mutex_;
    bool exinstruct_flag_;



    // For Main Program
    std::string func_name_;
    // For ama_func
    std::string move_name_;


    // For Threads related to movement....shared them with ama_waypoint
    bool change_func_flag_, change_goal_flag_, end_flag_;
    boost::recursive_mutex movement_mutex_;








  };

};

#endif
