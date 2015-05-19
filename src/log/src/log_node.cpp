/**
 * Read a messages from other nodes and plot information
 */


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

//STD
#include <iostream>
#include <fstream>
#include <vector>

//Robot
#include <pa10controller/getJointConfig.h>

using namespace std;

//ROS Paths
#define SUB_BALL "balltracker/points"
#define SUB_KALMAN "kalman_filter/points"
#define SUB_CAMERA_POS "path_planning/camera_transformation"
#define SUB_CAMERA_POS_REAL "path_planning/camera_transformation"
#define SRV_GET_JOINT "/getJointConfig"
#define PARAM_DEBUGGING "/points_server/debugging"
#define PARAM_FRAME_RATE "/frame_rate"

geometry_msgs::PointStamped_<float> ball_point;
geometry_msgs::PointStamped_<float> kalman_point;
geometry_msgs::PoseStamped_<float> camera_pose;
geometry_msgs::PoseStamped_<float> camera_pose_real;

void callbackBall(geometry_msgs::PointStamped_<float> point){
	ball_point.header.frame_id = point.header.frame_id;
	ball_point.header.seq = point.header.seq;
	ball_point.header.stamp = point.header.stamp;
	ball_point.point.x = point.point.x;
	ball_point.point.y = point.point.y;
	ball_point.point.z = point.point.z;
}

void callbackKalman(geometry_msgs::PointStamped_<float> point){
	kalman_point.header.frame_id = point.header.frame_id;
	kalman_point.header.seq = point.header.seq;
	kalman_point.header.stamp = point.header.stamp;
	kalman_point.point.x = point.point.x;
	kalman_point.point.y = point.point.y;
	kalman_point.point.z = point.point.z;
}

void callbackCameraPose(geometry_msgs::PoseStamped_<float> pose){
	camera_pose.header.frame_id = pose.header.frame_id;
	camera_pose.header.seq = pose.header.seq;
	camera_pose.header.stamp = pose.header.stamp;
	camera_pose.pose.orientation.x = pose.pose.orientation.x;
	camera_pose.pose.orientation.y = pose.pose.orientation.y;
	camera_pose.pose.orientation.z = pose.pose.orientation.z;
	camera_pose.pose.orientation.w = pose.pose.orientation.w;
	camera_pose.pose.position.x = pose.pose.position.x;
	camera_pose.pose.position.y = pose.pose.position.y;
	camera_pose.pose.position.z = pose.pose.position.z;
}

void callbackCameraPoseReal(geometry_msgs::PoseStamped_<float> pose){
	camera_pose_real.header.frame_id = pose.header.frame_id;
	camera_pose_real.header.seq = pose.header.seq;
	camera_pose_real.header.stamp = pose.header.stamp;
	camera_pose_real.pose.orientation.x = pose.pose.orientation.x;
	camera_pose_real.pose.orientation.y = pose.pose.orientation.y;
	camera_pose_real.pose.orientation.z = pose.pose.orientation.z;
	camera_pose_real.pose.orientation.w = pose.pose.orientation.w;
	camera_pose_real.pose.position.x = pose.pose.position.x;
	camera_pose_real.pose.position.y = pose.pose.position.y;
	camera_pose_real.pose.position.z = pose.pose.position.z;
}

int main(int argc, char **argv)
{
	ROS_INFO("Log Started!");
	ros::init(argc, argv, "log_node");

	//ROS
	ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber(SUB_BALL, 1, callbackBall);
	ros::Subscriber(SUB_BALL, 1, callbackKalman);
	ros::Subscriber(SUB_BALL, 1, callbackCameraPose);
	ros::Subscriber(SUB_BALL, 1, callbackCameraPoseReal);
	//Services
	ros::ServiceClient client_getJointConfig =
			nh.serviceClient<pa10controller::getJointConfig>(SRV_GET_JOINT);
	pa10controller::getJointConfig srv_getJointConfig;

	//Debugging parameter
	bool debugging = true;
	nh.setParam(PARAM_DEBUGGING, true);

	//Point to store the prediction
	geometry_msgs::PointStamped point_file;

	//File
	ofstream file;
	string line;

	if (!file.is_open()) cout << "File not found!" << endl;

	//WallTime
	ros::WallTime walltime;
	int frame_rate = 1;

	while(ros::ok() && file.is_open()){
		//Debugging
		ros::param::get(PARAM_DEBUGGING, debugging);
		//Frame Rate
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);

		//Q real robot
		float Q_real[7];
		if (client_getJointConfig.call(srv_getJointConfig)){
			for (unsigned char joint = 0; joint < 7; ++joint)
				Q_real[joint] = srv_getJointConfig.response.positions[joint];
		}

		//And sleep
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
