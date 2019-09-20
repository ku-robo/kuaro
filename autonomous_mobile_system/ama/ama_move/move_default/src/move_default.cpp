
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.h>
#include <move_default/move_default.hpp>


//register this mode as a AMAMove plugin
PLUGINLIB_EXPORT_CLASS(move_default::MoveDefault, ama_core::AMAMove)

namespace move_default{
  MoveDefault::MoveDefault(void):
  initialize_flag_(false), first_flag_(true), movegoal_flag_ptr_(NULL), break_flag_ptr_(NULL), aborted_flag_ptr_(NULL), exinstruct_flag_ptr_(NULL),movegoal_next_g_ptr_(NULL),
  current_status_(" NOTHING ")
  {}

  MoveDefault::MoveDefault(std::string name, ama_struct::GetPoint* movegoal_next_g, bool* movegoal_flag, bool* break_flag, bool* aborted_flag,  bool* exinstruct_flag):
  initialize_flag_(false), first_flag_(true), movegoal_flag_ptr_(NULL), break_flag_ptr_(NULL), aborted_flag_ptr_(NULL), exinstruct_flag_ptr_(NULL),movegoal_next_g_ptr_(NULL),
  current_status_(" NOTHING ")
  {
    initialize(name, movegoal_next_g, movegoal_flag, break_flag, aborted_flag, exinstruct_flag);
  }

  MoveDefault::~MoveDefault(void){}

  void MoveDefault::initialize(std::string name, ama_struct::GetPoint* movegoal_next_g, bool* movegoal_flag, bool* break_flag, bool* aborted_flag,  bool* exinstruct_flag)
  {
    if(!initialize_flag_){
      movegoal_next_g_ptr_ = movegoal_next_g;
      movegoal_flag_ptr_ = movegoal_flag;
      break_flag_ptr_ = break_flag;
      aborted_flag_ptr_ = aborted_flag;
      exinstruct_flag_ptr_ = exinstruct_flag;
      ros::NodeHandle private_nh("~/"+name);
      private_nh.param("max_aborted_count", max_aborted_count_, 10 );

      ros::NodeHandle mwf_nh("mwf");

      pub_mb_status_ = mwf_nh.advertise<std_msgs::String>("move_base_status", 1);
      initialize_flag_ = true;
    }else{
      ROS_WARN(" [AMAMovePlugin/MoveDefault][Func:initialize] This planner has already been initialized... doing nothing");
    }
  }

  bool MoveDefault::move_work(boost::recursive_mutex& movegoal_mutex, boost::recursive_mutex& break_mutex, boost::recursive_mutex& aborted_mutex, boost::recursive_mutex& exinstruct_mutex)
  {
    static MoveBaseClient ac("move_base", true);
    static unsigned int aborted_count = 0;
    if(first_flag_){
      // ac.getStateによって得られる値 ↓
      // PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

      // クライアント・サービスが立ち上がるまで待機する
      while(!ac.waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }
    }
    ros::Rate exinstr_loop(1);
    ros::Rate first_loop(5);
    ros::Rate loop(10);
    while(ros::ok()){

      // CHECK WETHER TO BREAKING
      // check if end flag is true. if true, return
      boost::unique_lock< boost::recursive_mutex > break_l(break_mutex);
      if( (*break_flag_ptr_) ){
        break_l.unlock();
        ac.cancelAllGoals();
        ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] STOP&BREAK!!!  Cancel All Goals");
        return true;
      }else{
        break_l.unlock();
        ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] CONTINUE!!!");
      }

      // CHECK FOR NEW WAYPOINT
      // check if func flag is true. if true, change running function.
      boost::unique_lock< boost::recursive_mutex > movegoal_l(movegoal_mutex);
      if( *movegoal_flag_ptr_ ){
        *movegoal_flag_ptr_ = false;
        current_goal_ = (*movegoal_next_g_ptr_).MoveBaseGoal();
        movegoal_l.unlock();
        ac.sendGoal(current_goal_);
        ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] GO!!! Towards the NEXT WayPoint");
      }else{
        movegoal_l.unlock();
        ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] GO!!! Towards the CURRENT WayPoint");
      }

      // CHECK FOR EXTERNAL OPERATION
      // if true, external operation will stop moving, no if it will restart moving
      boost::unique_lock< boost::recursive_mutex > exinst_l(exinstruct_mutex);
      if(*exinstruct_flag_ptr_){
        exinst_l.unlock();
        ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] EXTERNAL OPERATION !!!~STOP~!!! ");
        ac.cancelAllGoals();
        while(ros::ok()){
          exinst_l.lock();
          if(!(*exinstruct_flag_ptr_) ){
            exinst_l.unlock();
            ROS_INFO(" [AMAMovePlugin/MoveDefault][Func:move_work] EXTERNAL OPERATION !!!~GO~!!! ");
            ac.sendGoal(current_goal_);
            break;
          }
          exinst_l.unlock();
          exinstr_loop.sleep();
        }
      }

      actionlib::SimpleClientGoalState state = ac.getState();
      // check move base status and When the conditions are met, publish data.
      if(state == actionlib::SimpleClientGoalState::ABORTED){
        publish_mb_status(" ABORTED ");
        aborted_count++;
        if( max_aborted_count_ < aborted_count ){
          aborted_count = 0;
          boost::recursive_mutex::scoped_lock aborted_l(aborted_mutex);
          *aborted_flag_ptr_ = true;
        }
      }else{
        aborted_count = 0;
        if(state == actionlib::SimpleClientGoalState::PENDING){
          publish_mb_status(" PENDING ");
        }else if(state == actionlib::SimpleClientGoalState::RECALLED){
          publish_mb_status(" RECALLED ");
        }else if(state == actionlib::SimpleClientGoalState::PREEMPTED){
          publish_mb_status(" PREEMPTED ");
        }else if(state == actionlib::SimpleClientGoalState::ACTIVE){
          publish_mb_status(" ACTIVE ");
        }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
          publish_mb_status(" SUCCEEDED ");
        }else if(state == actionlib::SimpleClientGoalState::LOST){
          publish_mb_status(" LOST ");
        }
      }

      // sleep
      loop.sleep();
    }
  }

  void MoveDefault::publish_mb_status(const std::string current_status)
  {
    std_msgs::String pub_mb_status_str;
    // check if current_status_ and current_status are same value.
    // if not, it means current_status have new status.
    if(current_status_ != current_status){
      current_status_ = current_status;
      pub_mb_status_str.data = current_status_;
      pub_mb_status_.publish(pub_mb_status_str);
    }
  }

};
