// メンバ変数には_を最後につけること

#ifndef FUNC_DEFAULT
#define FUNC_DEFAULT

// ros
#include <ros/ros.h>
// self-made
#include <ama_core/ama_func.hpp>


namespace func_default{

  class FuncDefault : public ama_core::AMAFunc{
  public:

    FuncDefault(void);
    FuncDefault(std::string name, tf::TransformListener* tf);
    /*
    * @brief  Initialization function for the AMAFunc
    */
    void initialize(std::string name, tf::TransformListener* tf);
    /*
    * @brief this is a characteristic of each mode
    */
    bool func_work();
    /*
    * @brief
    */
    bool update_flag(void);
    /*
    * @brief
    */
    bool maintain_flag(void);
    /*
    * @brief
    */
    std::vector<ama_struct::GetPoint> GetPoints(void);
    /*
    * @brief return the used move method name
    */
    std::string get_move_method(void);


    ~FuncDefault(void);
  private:


    // Must initialize
    bool initialize_flag_, change_flag_, maintain_flag_;
    tf::TransformListener* tf_;
    std::vector<ama_struct::GetPoint> waypoint_vec_;
    std::string move_method_;


  };

};

#endif
