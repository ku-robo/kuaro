/*-----------------------------------------------
* 	person_manager.cpp
* <Last Update>	H29/09/05
* <version>		v1.0
*
* <MEMO>
*
* ---------------------------------------------*/

#include "stdafx.hpp"
#include "person_manager.hpp"

/*-----MAIN--------------------------------------*/
int main(int argc,char *argv[])
{
	ros::init(argc, argv, "person_manager");

	PERSON_MANAGER manager;

	ros::NodeHandle perosn_node;
	ros::Subscriber person_sub = perosn_node.subscribe("/person_point", 100, &PERSON_MANAGER::goal_send_callback, &manager);

	ros::spin();
	return 0;
}
