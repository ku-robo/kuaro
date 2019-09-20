/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/31
* <version>		v2.2
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include <2018_tsukuba_challenge/stdafx.hpp>
#include "fake_obstacle.hpp"


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "person_manager");

	C_fake_obstacle shimabara;

	shimabara.run();

	return 0;
}
