// #include "youbot/YouBotBase.hpp"
// #include "youbot/YouBotManipulator.hpp"

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
//#include <io.h>
#include <fstream>
#include <string>
#include <sstream>

#include "youbot_driver/youbot/YouBotBase.hpp"

#include <youbot_camera/RT_youbot_base.h>

using namespace youbot;
using namespace youbot_camera_suite;

#define YOUBOT_CONFIGURATIONS_DIR "/home/sees/CommLineProg/nonproProg/robotics/catkin_ws/src/kuka-youbot-driver/config/"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "youbot_sub_pub");

	bool youBotHasBase;

	youbot_camera_suite::RT_youbot_base* robot;

	try 
	{
		robot = new youbot_camera_suite::RT_youbot_base("myRobot", YOUBOT_CONFIGURATIONS_DIR);
		youBotHasBase = true;
		ros::Rate loop_rate(10);

		while (ros::ok())
		{
			robot->update();

			ros::spinOnce();

			loop_rate.sleep();
		}
	}
	catch (std::exception& e)
	{
		LOG(warning) << e.what();
		youBotHasBase = false;

		/* clean up */
		delete robot;
		robot = 0;

		return 6;
	}
	youBotHasBase = false;


	/* clean up */
	if (youBotHasBase) {
		delete robot;
		robot = 0;
	}

	LOG(info) << "Done.";

	return 0;
}

