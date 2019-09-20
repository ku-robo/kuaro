#include <ama_main/ama_main.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "ama_main");

  tf::TransformListener tf(ros::Duration(10));


  ama_main::AMAMain ama_main_(tf);


  return 0;
}
