// メンバ変数には_を最後につけること

#ifndef AMA_GET_POINT
#define AMA_GET_POINT

// Standard
#include <cmath>
// ros
#include <ros/ros.h>
// ros-msgs
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
// tf
#include <tf/transform_datatypes.h>


namespace ama_struct{

  struct GetPoint{
    // public...struct member
    std::string frame_id;
    ros::Time stamp;
    float position_x;
    float position_y;
    float position_z;
    float orientation_x;
    float orientation_y;
    float orientation_z;
	  float orientation_w;
    float yaw;

    /*
    * @brief Constructor for GetPoint
    */
    GetPoint(void);
    /*
    * @brief Constructor for GetPoint
    */
    GetPoint(const std::string& thisframe_id);
    /*
    * @brief Constructor for GetPoint
    */
    GetPoint(const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief Change move_base_msgs::MoveBaseGoal to GetPoint Constructor for GetPoint
    * @param goal: set goal to GetPoint
    */
    GetPoint(const move_base_msgs::MoveBaseGoal& goal);
    /*
    * @brief Change move_base_msgs::MoveBaseGoal to GetPoint Constructor for GetPoint
    * @param
    */
    GetPoint(const move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief Change move_base_msgs::MoveBaseGoal to GetPoint Constructor for GetPoint
    * @param goal: set goal to GetPoint
    */
    GetPoint(const geometry_msgs::PoseStamped& g_pose);
    /*
    * @brief Change move_base_msgs::MoveBaseGoal to GetPoint Constructor for GetPoint
    * @param
    */
    GetPoint(const geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief Copy Constructor for GetPoint
    * @param gp: value to Copy
    */
    GetPoint(const GetPoint& gp);
    /*
    * @brief
    */
    void create_empty(void);
    /*
    * @brief
    */
    void create_equivalence( const float& equivalence );
    /*
    * @brief
    */
    void set_other(const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief
    */
    void allset_position( const float& equivalence );
    /*
    * @brief
    */
    void set_position( const float& x_val = 0.0, const float& y_val = 0.0, const float& z_val = 0.0);
    /*
    * @brief
    */
    void allset_orientation( const float& equivalence );
    /*
    * @brief
    */
    void set_orientation( const float& x_val = 0.0, const float& y_val = 0.0, const float& z_val = 0.0, const float& w_val = 0.0);
    /*
    * @brief
    */
    float gp_atan2(const float& y_val, const float& x_val);
    /*
    * @brief set Yaw value to orientations by arctan calcurated from the X-axis value and the Y-axis value.
    */
    void set_orientation_Yonly_byXY(const float& x_val, const float& y_val);
    /*
    * @brief
    */
    void set_orientation_Yonly_byPosiiton(void);
    /*
    * @brief
    */
    float InverseYaw(void);
    /*
    * @brief
    */
    void set_orientation_InverseYonly_byXY(const float& x_val, const float& y_val);
    /*
    * @brief
    */
    void set_orientation_InverseYonly_byPosiiton(void);
    /*
    * @brief set only the Yaw value to orientation.
    */
    void set_orientation_Yonly(const float& yaw_val);
    /*
    * @brief orientations are set by the Roll and Pitch, Yaw values
    */
    void set_orientation_RPY( const float& roll_val, const float& pitch_val, const float& yaw_val);
    /*
    * @brief
    */
    void set(const move_base_msgs::MoveBaseGoal& goal);
    /*
    * @brief
    */
    void set(const move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief
    */
    void set(const geometry_msgs::PoseStamped& g_pose);
    /*
    * @brief
    */
    void set(const geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime);
    /*
    * @brief
    */
    void get(move_base_msgs::MoveBaseGoal& goal) const;
    /*
    * @brief
    */
    void get(move_base_msgs::MoveBaseGoal& goal, const std::string& thisframe_id, const ros::Time& yourtime) const;
    /*
    * @brief
    */
    void get(geometry_msgs::PoseStamped& g_pose) const;
    /*
    * @brief
    */
    void get(geometry_msgs::PoseStamped& g_pose, const std::string& thisframe_id, const ros::Time& yourtime) const;
    /*
    * @brief
    */
    geometry_msgs::PoseStamped geometryPoseStamped(void) const;
    /*
    * @brief
    */
    geometry_msgs::PoseStamped geometryPoseStamped(const std::string& thisframe_id, const ros::Time& yourtime) const;
    /*
    * @brief
    */
    move_base_msgs::MoveBaseGoal MoveBaseGoal(void) const;
    /*
    * @brief
    */
    move_base_msgs::MoveBaseGoal MoveBaseGoal(const std::string& thisframe_id, const ros::Time& yourtime) const;
    /*
    * @brief
    */
    GetPoint differ(const GetPoint& gp) const;
    /*
    * @brief
    */
    GetPoint differ(const geometry_msgs::PoseStamped& gmps) const;
    /*
    * @brief
    */
    GetPoint differ(const move_base_msgs::MoveBaseGoal& mbg) const;
    /*
    * @brief
    */
    geometry_msgs::PoseStamped differ_geometryPoseStamped(const GetPoint& gp);
    /*
    * @brief
    */
    geometry_msgs::PoseStamped differ_geometryPoseStamped(const geometry_msgs::PoseStamped& gmps);
    /*
    * @brief
    */
    geometry_msgs::PoseStamped differ_geometryPoseStamped(const move_base_msgs::MoveBaseGoal& mbg );
    /*
    * @brief
    */
    move_base_msgs::MoveBaseGoal differ_MoveBaseGoal(const GetPoint& gp);
    /*
    * @brief
    */
    move_base_msgs::MoveBaseGoal differ_MoveBaseGoal(const geometry_msgs::PoseStamped& gmps);
    /*
    * @brief
    */
    move_base_msgs::MoveBaseGoal differ_MoveBaseGoal(const move_base_msgs::MoveBaseGoal& mbg);
    /*
    * @brief return hypot about this->position_x and this->position_y.
    */
    float xy_hypot(void);
    /*
    * @brief return hypot about differ between *this and gp.
    */
    float xy_hypot(const GetPoint& gp);
    /*
    * @brief return hypot about differ between *this and gmps.
    */
    float xy_hypot(const geometry_msgs::PoseStamped& gmps);
    /*
    * @brief return hypot about differ between *this and mbg.
    */
    float xy_hypot(const move_base_msgs::MoveBaseGoal& mbg );
    /*
    * @brief
    */
    GetPoint& xy_flexibility(const float& distination_hypot);
    /*
    * @brief
    */
    GetPoint& xy_flexibility(const float& original_hypot, const float& distination_hypot);
    /*
    * @brief
    */
    GetPoint& xy_ratio_flexibility(const float& ratio);
    // Operator
    /*
    * @brief
    */
    GetPoint& operator =(const GetPoint& gp) &;
    /*
    * @brief
    */
    GetPoint& operator =(const GetPoint&& gp) & noexcept;
    /*
    * @brief
    */
    GetPoint& operator =(const move_base_msgs::MoveBaseGoal& goal) &;
    /*
    * @brief
    */
    GetPoint& operator =(const move_base_msgs::MoveBaseGoal&& goal) & noexcept;
    /*
    * @brief
    */
    GetPoint& operator =(const geometry_msgs::PoseStamped& g_pose) &;
    /*
    * @brief
    */
    GetPoint& operator =(const geometry_msgs::PoseStamped&& g_pose) & noexcept;

    /*
    * @brief
    */
    GetPoint& operator+=(const GetPoint& gp);
    /*
    * @brief
    */
    GetPoint& operator+=(const move_base_msgs::MoveBaseGoal& goal);
    /*
    * @brief
    */
    GetPoint& operator+=(const geometry_msgs::PoseStamped& g_pose);
    /*
    * @brief
    */
    GetPoint& operator-=(const GetPoint& gp);
    /*
    * @brief
    */
    GetPoint& operator-=(const move_base_msgs::MoveBaseGoal& goal);
    /*
    * @brief
    */
    GetPoint& operator-=(const geometry_msgs::PoseStamped& g_pose);
    /*
    * @brief
    */
    virtual ~GetPoint(void);
  private:

  };

};

#endif
