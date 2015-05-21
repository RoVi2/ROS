/**
 * Read a file with 3D points and publish them
 */


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

//STD
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

//ROS Paths
#define TOPIC "/balltracker/points"
#define PARAM_DEBUGGING "/balltracker/debugging"
#define PARAM_FRAME_RATE "/frame_rate"
#define FILE_PATH "res/points.txt"

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

	int frame_rate = 5;
	nh.setParam(PARAM_FRAME_RATE, frame_rate);

	//Point to store the prediction
	geometry_msgs::PointStamped point_file;
	//Publisher for the point
	point_pub = nh.advertise<geometry_msgs::PointStamped>(TOPIC, 1);

	//File
	ifstream file;
	string line;
	file.open(FILE_PATH);

	if (!file.is_open()) cout << "File not found!" << endl;

	//WallTime
	ros::WallTime walltime;

	while(ros::ok() && file.is_open()){
		//Debugging
		ros::param::get(PARAM_DEBUGGING, debugging);
		//Frame Rate
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);
		//Get next line from the file
		if (getline(file, line))
		{
			//Read a line and store its values
			istringstream istr(line);
			istr >> point_file.point.x >> point_file.point.y >> point_file.point.z;
			point_file.header.frame_id = "WORLD";
			point_file.header.stamp.sec = walltime.now().sec;
			if (debugging) cout << "X:" << point_file.point.x << " Y:" << point_file.point.y << " Z:" << point_file.point.z
					<< " T: " << point_file.header.stamp << endl;
			//And now we publish the point
			point_pub.publish(point_file);

		}
		//If we have reached the end of the file, we start again
		else
		{
			file.clear();
			file.seekg(0, ios::beg);
		}
		//And sleep
		loop_rate.sleep();
	}
	return 0;
}
