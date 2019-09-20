// メンバ変数には_を最後につけること

#ifndef AMA_FUNC
#define AMA_FUNC

// Standard
#include <stdio.h>
#include <string>
// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
// self-made
#include <ama_struct/get_point.hpp>


namespace ama_core{

  // 絶対座標系(/map)になおしてからvectorに入れること！！！

  class AMAFunc{
  public:
    /*
    * @brief  Initialization function for the AMAFunc
    */
    virtual void initialize(std::string name, tf::TransformListener* tf) = 0;
    /*
    * @brief this is a characteristic of each mode
    */
    virtual bool func_work(void)=0;
    /*
    * @brief
    */
    virtual bool update_flag(void)=0;
    /*
    * @brief
    */
    virtual bool maintain_flag(void)=0;
    /*
    * @brief
    */
    virtual std::vector<ama_struct::GetPoint> GetPoints(void)=0;
    /*
    * @brief return the used move method name
    */
    virtual std::string get_move_method(void)=0;
    /*
    * @brief destractor
    */
    virtual ~AMAFunc(void){}
  protected:
    /*
    * @brief constractor
    */
    AMAFunc(void){}
  };

};

#endif
