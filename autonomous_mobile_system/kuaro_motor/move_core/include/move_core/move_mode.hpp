#ifndef MOVE_CORE
#define MOVE_CORE

#include <stdio.h>
#include <ambi_core/core_parts.hpp>

namespace move_core{

  class MoveMode {
  public:
    /*
    * @brief set up subscriber and set params
    * @param name : using for node name
     */
    virtual void initialize(std::string name)=0;
    /*
    * @brief give you commands this class sets.
     */
    virtual ambi_core::TwistParam get_command_value(void)=0;
    /*
    * @brief give you commands this class sets.
     */
    virtual void use_encode_info(const ambi_core::PoseParam& pose_info, const ambi_core::TwistParam& twist_info)=0;
    /*
    * @brief check whether there was an update.
     */
    virtual bool check_update_flag(bool change_flag)=0;
    /*
    * @brief check encode_flag_'s value.
     */
    virtual bool check_encode_flag(void){
      return false;
    }

    virtual ~MoveMode(void){}
  protected:
    MoveMode(void){}

    ambi_core::TwistParam send_twist_;
  };

};

#endif
