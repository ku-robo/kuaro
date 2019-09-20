/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/10/24
* <version>		v2.0
*
* <MEMO>v1.0を元に佐藤が編集したものを秋本が編集
*
* ---------------------------------------------*/

#include "stdafx.hpp"
#include "person_manager.hpp"


int main(int argc,char *argv[])
{
	ros::init(argc, argv, "person_manager");

	C_person_manager manager;

	manager.run();

	return 0;
}
