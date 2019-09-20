#include <kuaro_mover/kuaro_mover.hpp>

int main(int argc, char** argv){
  ros::init(argc, argv, "kuaro_mover_node");

  kuaro_mover::KuaroMover kuaro_move;
  ROS_INFO(" [kuaro_mover_node] kuaro_mover_node start...");
  kuaro_move.run();

  return 0;
}
