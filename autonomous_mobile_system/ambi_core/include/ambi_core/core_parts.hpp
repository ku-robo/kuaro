#ifndef CORE_PARTS
#define CORE_PARTS

#include <stdio.h>


namespace ambi_core{

  struct PoseParam
  {
    double posi_x;
    double posi_y;
    double posi_z;
    double orien_x;
    double orien_y;
    double orien_z;
    double orien_w;

    PoseParam(void){}
    PoseParam(double init_val){
      init(init_val);
    }
    PoseParam(const PoseParam& pose_data){
      posi_x = pose_data.posi_x;
      posi_y = pose_data.posi_y;
      posi_z = pose_data.posi_z;
      orien_x = pose_data.orien_x;
      orien_y = pose_data.orien_y;
      orien_z = pose_data.orien_z;
      orien_w = pose_data.orien_w;
    }

    PoseParam& operator = (const PoseParam& pose_data){
      this->posi_x = pose_data.posi_x;
      this->posi_y = pose_data.posi_y;
      this->posi_z = pose_data.posi_z;
      this->orien_x = pose_data.orien_x;
      this->orien_y = pose_data.orien_y;
      this->orien_z = pose_data.orien_z;
      this->orien_w = pose_data.orien_w;
      return *this;
    }
    void init(double init_val){
      double list[7] = {posi_x, posi_y, posi_z, orien_x, orien_y, orien_z, orien_w};
      for(int i = 0; i < 7 ; i++){
        list[i] = init_val;
      }
    }
  };

  struct TwistParam
  {
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;

    TwistParam(void){}
    TwistParam(double init_val){
      init(init_val);
    }
    TwistParam(const TwistParam& twist_data){
      linear_x = twist_data.linear_x;
      linear_y = twist_data.linear_y;
      linear_z = twist_data.linear_z;
      angular_x = twist_data.angular_x;
      angular_y = twist_data.angular_y;
      angular_z = twist_data.angular_z;
    }

    TwistParam& operator = (const TwistParam& twist_data){
      this->linear_x = twist_data.linear_x;
      this->linear_y = twist_data.linear_y;
      this->linear_z = twist_data.linear_z;
      this->angular_x = twist_data.angular_x;
      this->angular_y = twist_data.angular_y;
      this->angular_z = twist_data.angular_z;
      return *this;
    }
    void init(double init_val){
      double list[6] = {linear_x, linear_y, linear_z, angular_x, angular_y, angular_z};
      for(int i = 0; i < 6; i++){
        list[i] = init_val;
      }
    }
  };

};

#endif
