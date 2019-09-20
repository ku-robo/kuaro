#include "stdafx.hpp"
#include "camera_lrf_img_comb.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_comb_subscriber");
	C_camera_comb camera_comb;

	std::cout << "start" << std::endl;

	camera_comb.run();

	std::cout << "end" << std::endl;

	return 0;
}
