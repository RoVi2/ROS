#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cereal_port/CerealPort.h>
#include <iostream>

#define REPLY_SIZE 3
#define TIMEOUT 2000


using namespace std;
cereal::CerealPort device;




int main(int argc, char **argv)
{
	ros::init(argc, argv, "pa10_server");
	if (argc != 2)
	{
		ROS_INFO("usage: pa10_server port");
		return 1;
	}

	ros::NodeHandle n;

	char * portname = argv[1];

	ROS_INFO("Starting PA10 control on %s",portname);
    try{ device.open(portname, 115200); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");

	ros::spin();

	return 0;
}
