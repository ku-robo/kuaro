#ifndef MOVE_SAFETY
#define MOVE_SAFETY

#include <stdio.h>
#include <ambi_core/core_parts.hpp>

namespace move_core{

  class MoveSafety {
  public:
    /*
    * @brief set up subscriber and set params
    * @param name : using for node name
     */
    virtual void initialize(std::string name)=0;

    virtual void change_safety_value( ambi_core::TwistParam& input)=0;

    virtual ~MoveSafety(){}
  protected:
    MoveSafety(){}

  };

};

#endif
