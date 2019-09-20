////////////////////////////////////////////////////////////////////////////////
//
// 作成開始日 2018 09 06 (木)
// ver.1完成日
//
//
//
// 作成者 : 福田　優人
//
//
////////////////////////////////////////////////////////////////////////////////


#include <2018_tsukuba_challenge/Robot_goals_decider.hpp>


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "Robot_goals_decider");



	Robot_goals_decider Robot_G;
	Robot_G.main_mover();


	return 0;
}
