/**
 * Read a file with 3D points and publish them
 */


//ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

//STD
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

//ROS Paths
#define TOPIC "/points_server/points"
#define PARAM_DEBUGGING "/points_server/debugging"
#define FILE_PATH "../../../src/points_server/res/points.txt"

int main(int argc, char **argv)
{
	ROS_INFO("Points Server Started!");
	ros::init(argc, argv, "points_server_node");

	//ROS
	ros::NodeHandle nh;
	ros::Publisher point_pub;

	//Debugging parameter
	bool debugging = false;
	nh.setParam(PARAM_DEBUGGING, false);

	//Point to store the prediction
	geometry_msgs::Point point_file;
	//Publisher for the point
	point_pub = nh.advertise<geometry_msgs::Point>(TOPIC, 1);

	//File
	ifstream file;
	string line;
	file.open(FILE_PATH);

	if (file.is_open() && debugging) cout << "File found!" << endl;

	/*
	 * Refresh frequency
	 */
	ros::Rate loop_rate(1);

	while(ros::ok() && file.is_open()){
		ros::param::get(PARAM_DEBUGGING, debugging);
		if (getline(file, line))
		{
			//Read a line and store its values
			istringstream istr(line);
			istr >> point_file.x >> point_file.y >> point_file.z;
			if (debugging) cout << "X:" << point_file.x << " Y:" << point_file.y << " Z:" << point_file.z << endl;
			//And now we publish the point
			point_pub.publish(point_file);

		}
		//If we have reached the end of the file, we start again
		else
		{
			file.clear();
			file.seekg(0, ios::beg);
		}
		//And Spin!
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
