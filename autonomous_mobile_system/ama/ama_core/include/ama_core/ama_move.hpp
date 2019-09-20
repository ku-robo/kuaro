// メンバ変数には_を最後につけること

#ifndef AMA_MOVE
#define AMA_MOVE

// Standard
#include <stdio.h>
#include <string>
// Boost
#include <boost/thread.hpp>
// self-made
#include <ama_struct/get_point.hpp>

namespace ama_core{

  class AMAMove{
  public:
    /*
    * @brief  Initialization function for the AMAFunc
    * @param
    */
    virtual void initialize(std::string name, ama_struct::GetPoint* movegoal_next_g, bool* movegoal_flag, bool* break_flag, bool* aborted_flag,  bool* exinstruct_flag) = 0;
    /*
    * @brief this is a characteristic of each mode .... if returned value is true, it means all process is end.
    */
    virtual bool move_work(boost::recursive_mutex& movegoal_mutex, boost::recursive_mutex& break_mutex, boost::recursive_mutex& aborted_mutex, boost::recursive_mutex& exinstruct_mutex)=0;

    

    /*
    * @brief destractor
    */
    virtual ~AMAMove(void){}
  protected:
    /*
    * @brief constractor
    */
    AMAMove(void){}
  };

};

#endif
