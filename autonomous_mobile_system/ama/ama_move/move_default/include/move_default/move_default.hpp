// メンバ変数には_を最後につけること

#ifndef MOVE_DEFAULT
#define MOVE_DEFAULT

// ros
#include <ros/ros.h>
// ros-msgs
#include <std_msgs/String.h>
// Boost
#include <boost/thread.hpp>
// self-made
#include <ama_core/ama_move.hpp>
// move_base_msgs
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


namespace move_default{
	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  class MoveDefault : public ama_core::AMAMove{
  public:
		/*
    * @brief
    */
		MoveDefault(void);
		/*
    * @brief
    */
		MoveDefault(std::string name, ama_struct::GetPoint* movegoal_next_g, bool* movegoal_flag, bool* break_flag, bool* aborted_flag,  bool* exinstruct_flag);
    /**
    * @brief  Initialization function for the AMAFunc
    * @param
    */
    void initialize(std::string name, ama_struct::GetPoint* movegoal_next_g, bool* movegoal_flag, bool* break_flag, bool* aborted_flag,  bool* exinstruct_flag);
    /*
    * @brief this is a characteristic of each mode
    */
    bool move_work(boost::recursive_mutex& movegoal_mutex, boost::recursive_mutex& break_mutex, boost::recursive_mutex& aborted_mutex, boost::recursive_mutex& exinstruct_mutex);
    /*
    * @brief destractor
    */
    ~MoveDefault(void);
  private:
    /*
    * @brief
    */
    void publish_mb_status(const std::string current_status);

    // Must initialize
    bool initialize_flag_, first_flag_;
    bool* movegoal_flag_ptr_;
		bool* break_flag_ptr_;
		bool* aborted_flag_ptr_;
		bool* exinstruct_flag_ptr_;
    ama_struct::GetPoint* movegoal_next_g_ptr_;

		int max_aborted_count_;
    move_base_msgs::MoveBaseGoal current_goal_;


    // for checking move base status
    std::string current_status_;
		// ros
    ros::Publisher pub_mb_status_;
  };

};

#endif
