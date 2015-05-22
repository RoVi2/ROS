/*
 * Path Planning Node
 *
 *  Created on: Mar 28, 2015
 *      Author: Jorge Rodriguez Marin
 */

//STD
#include <iostream>

//RobWork
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_planning/Q_desired.h>
#include <path_planning/Q_real.h>

//Services
//#include <path_planning/getJointConfig.h>
//#include <path_planning/setJointConfig.h>
#include <pa10controller/getJointConfig.h>
#include <pa10controller/setJointConfig.h>
#include "pa10_dummy/getJointConfig.h"
#include "pa10_dummy/setJointConfig.h"

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

//Real Robot or Simulation
#define SIMULATION 0 //1=PA10_dummy 0=pa10controller

//RobWork Paths
#define WORKCELL_PATH "../../res/PA10Scene/sceneComplete.wc.xml"
#define ROBOT_NAME "PA10"

//ROS Paths
//#define SUBSCRIBER "/balltracker/points"
#define SUBSCRIBER "/kalman_filter/points"

#define PARAM_DEBUGGING "/path_planning/debugging"
#define PARAM_FRAME_RATE "/frame_rate"
#define PARAM_PATH_NOT_FOUND_TIME_LIMIT "/path_planning/path_not_found_time_limit"
#define PARAM_SPLINE_PATH_DT "/path_planning/spline_path_dt" //
#define PARAM_STEP_DT "/path_planning/step_dt" //For each time steps, how many times we split the path in
#define SRV_GET_JOINT "/getJointConfig"
#define SRV_SET_JOINT "/setJointsConfig"
#define MSG_Q_DESIRED "/path_planning/q_desired"
#define MSG_Q_REAL "/path_planning/q_real"
#define MSG_CAMERA_POSE_DESIRED "/path_planning/camera_pose_desired"
#define MSG_CAMERA_POSE_REAL "/path_planning/camera_pose_real"

//Colors
#define RESET "\e[m"
#define GREEN "\e[32m"
#define YELLOW "\e[33m"
#define MAGENTA "\e[35m"
#define CYAN "\e[36m"

//Global variables
Vector3D<double> point_kalman;
bool point_updated = false;
float x_offset = 0*Deg2Rad;
float y_offset = 90*Deg2Rad;
float z_offset = -57.6*Deg2Rad;

/**
 * Callback method to get the points from ROS
 */
void callBack(const geometry_msgs::PointStampedConstPtr & point_ros){
	point_kalman[0] = point_ros->point.x;
	point_kalman[1] = point_ros->point.y;
	point_kalman[2] = point_ros->point.z;
	point_updated = true;
}

#if (SIMULATION == 0)
/**
 * Sends the desired Q to the real robot
 * @param Q_desired
 * @param client_setJointConfig
 * @param srv_setJointConfig
 * @param debugging
 */
void sendRobotToQ(rw::math::Q & Q_desired,
		ros::ServiceClient & client_setJointConfig,
		pa10controller::setJointConfig & srv_setJointConfig,
		bool & debugging){
	//Copy the joint values in to the service message
	for (unsigned char joint=0; joint<7; joint++)
	{
		srv_setJointConfig.request.commands[joint] =  1;
		srv_setJointConfig.request.positions[joint] = Q_desired[joint]*Rad2Deg;
	}
	//Publish it!
	if (client_setJointConfig.call(srv_setJointConfig))
	{
		if (debugging) cout << "	" << Q_desired << endl;

		/*
		 * Maybe problems with sending too many Q? This is your place!
		 */
		Timer waitUntilRobot;
		waitUntilRobot.resetAndResume();
		while (waitUntilRobot.getTimeMs() <= 250)
			; //Do nothing
	}
	else if (debugging) ROS_ERROR("Robot not found");
}

#elif (SIMULATION == 1)
/**
 * Sends the desired Q to the dummy robot
 * @param Q_desired
 * @param client_setJointConfig
 * @param srv_setJointConfig
 * @param debugging
 */
void sendRobotToQ(rw::math::Q & Q_desired,
		ros::ServiceClient & client_setJointConfig,
		pa10_dummy::setJointConfig & srv_setJointConfig,
		bool & debugging){
	//Copy the joint values in to the service message
	for (unsigned char joint=0; joint<7; joint++)
	{
		srv_setJointConfig.request.commands[joint] =  1;
		srv_setJointConfig.request.positions[joint] = Q_desired[joint]*Rad2Deg;
	}
	//Publish it!
	if (client_setJointConfig.call(srv_setJointConfig))
	{
		if (debugging) cout << "	" << Q_desired << endl;

		/*
		 * Maybe problems with sending too many Q? This is your place!
		 */
		Timer waitUntilRobot;
		waitUntilRobot.resetAndResume();
		while (waitUntilRobot.getTimeMs() <= 250)
			; //Do nothing
	}
	else if (debugging) ROS_ERROR("Robot not found");
}
#endif

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
	/*
	 * ROS Stuff
	 */
	ROS_INFO("Path Planning Started!");
	ros::init(argc, argv, "path_planning_node");
	ros::NodeHandle nh;
	ros::Subscriber point_sub;

	//Parameters
	bool debugging = true;
	int frame_rate;
	double spline_path_dt = 3; //DONT PUT 1 OR LESS
	double step_dt = 1;

	nh.setParam(PARAM_DEBUGGING, true);
	nh.setParam(PARAM_SPLINE_PATH_DT, spline_path_dt);
	nh.setParam(PARAM_STEP_DT, step_dt);
	nh.getParam(PARAM_FRAME_RATE, frame_rate); //Obtained from the server

	//Get the new predicted point and store it
	point_sub = nh.subscribe(SUBSCRIBER, 1,  callBack);

	//Messages
	ros::Publisher pub_q_desired = nh.advertise<path_planning::Q_desired>(MSG_Q_DESIRED, 1);
	ros::Publisher pub_q_real = nh.advertise<path_planning::Q_real>(MSG_Q_REAL, 1);
	ros::Publisher pub_camera_pose_desired = nh.advertise<geometry_msgs::PoseStamped>(MSG_CAMERA_POSE_DESIRED, 1);
	ros::Publisher pub_camera_pose_real = nh.advertise<geometry_msgs::PoseStamped>(MSG_CAMERA_POSE_REAL, 1);

	//Services
#if SIMULATION==0
	ros::ServiceClient client_getJointConfig = nh.serviceClient<pa10controller::getJointConfig>(SRV_GET_JOINT);
	ros::ServiceClient client_setJointConfig = nh.serviceClient<pa10controller::setJointConfig>(SRV_SET_JOINT);
	pa10controller::getJointConfig srv_getJointConfig;
	pa10controller::setJointConfig srv_setJointConfig;
#elif SIMULATION==1
	ros::ServiceClient client_getJointConfig = nh.serviceClient<pa10_dummy::getJointConfig>(SRV_GET_JOINT);
	ros::ServiceClient client_setJointConfig = nh.serviceClient<pa10_dummy::setJointConfig>(SRV_SET_JOINT);
	pa10_dummy::getJointConfig srv_getJointConfig;
	pa10_dummy::setJointConfig srv_setJointConfig;
#endif
	/*
	 * RobWork
	 */
	//Load the workcell and the device
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(WORKCELL_PATH);
	Device::Ptr device = wc->findDevice(ROBOT_NAME);

	//Generate randomness for the RRT
	Math::seed();

	//Obtain the current state and position of the real robot
	State state = wc->getDefaultState();
	Q Q_ready = Q(7, 1.14, -0.65, 0, 1.8, 0, -0.96, 0.9);
	Q Q_current(7, 0,0,0,0,0,0,0);
	Timer pathNotFoundTimer;
	int pathNotFoundTimeLimit = 10;

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 0.3; //Distance to search the next point
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	//Going home
	sendRobotToQ(Q_ready, client_setJointConfig, srv_setJointConfig, debugging);
	device->setQ(Q_ready, state);

	Rotation3D<> R_x = Rotation3D<>(1,0,0,0,cos(x_offset),-sin(x_offset),0,sin(x_offset),cos(x_offset));
	Rotation3D<> R_y = Rotation3D<>(cos(y_offset),0,sin(y_offset),0,1,0,-sin(y_offset),0,cos(y_offset));
	Rotation3D<> R_z = Rotation3D<>(cos(z_offset),-sin(z_offset),0,sin(z_offset),cos(z_offset),0,0,0,1);
	RPY<> offset_for_worcell = RPY<>(R_y*R_z);
	cout << "//////////////////////////////////////////////////////////////////////////" << endl;
	cout << "Include this offset to the workcell: " << offset_for_worcell[0]*Rad2Deg << " " <<
			offset_for_worcell[1]*Rad2Deg << " " << offset_for_worcell[2]*Rad2Deg << endl;
	cout << "//////////////////////////////////////////////////////////////////////////" << endl;

	ros::WallTime walltime;

	/*
	 * Loop!
	 */
	while(ros::ok()){
		//Check if debugging
		nh.getParam(PARAM_DEBUGGING, debugging);
		//Refresh frequency
		nh.getParam(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);
		//Get the path not found time limit
		nh.getParam(PARAM_PATH_NOT_FOUND_TIME_LIMIT, pathNotFoundTimeLimit);
		//And the interpolation options
		nh.getParam(PARAM_SPLINE_PATH_DT, spline_path_dt);
		nh.getParam(PARAM_STEP_DT, step_dt);

		//Generate randomness
		Math::seed();

		//Obtain from the kalman point, the point where we want to put the camera.
		//This point for now will be that which is at 0.5.m from the object and is contained
		//	in (1) a plane that with the kalman point is parallel to the ground and (2) also
		//	in the plane generated by the robot's base axis and the kalman point.

		/*
		 * Obtains the Desired Q
		 */
		if(point_updated){
			//Translate the kalman point from the camera to the robot's base
			if (debugging) cout << endl << GREEN "New Point from: " RESET << endl;
			if (debugging) cout << GREEN"	Camera: " RESET << point_kalman << endl;
			//Updates the robot's position
			client_getJointConfig.call(srv_getJointConfig);
			for (unsigned char joint = 0; joint < 7; ++joint){
				Q_current[joint] = srv_getJointConfig.response.positions[joint]*Deg2Rad;
			}
			device->setQ(Q_current, state);
			//Reference the frame to the robot's base
			point_kalman = (device->baseTframe(wc->findFrame("Camera"), state)) * point_kalman;
			if (debugging) cout << GREEN"	Base: " RESET << point_kalman << endl;
			//Calculate in polar coordinates so it will simplify the calculations P(Radius, angle, height)
			double radius = sqrt(pow(point_kalman[0], 2) +  pow(point_kalman[1], 2));
			double angle = atan2(point_kalman[1],point_kalman[0]);
			double height = point_kalman[2];
			Vector3D<double> point_kalman_polar (radius, angle, height);
			if (debugging) cout << GREEN"	and in Polar: " RESET << point_kalman_polar << endl;
			//The camera position will be 0.5 meters from the kalman point in the radial direction
			Vector3D<double> camera_position_polar = point_kalman_polar - Vector3D<double>(0.4,0,0);
			Vector3D<double> camera_position (
					camera_position_polar[0]*cos(camera_position_polar[1]), //r*cos(angle)
					camera_position_polar[0]*sin(camera_position_polar[1]), //r*sin(angle)
					camera_position_polar[2] //height
			);
			//The rotation has to be that which is oriented to the point.
			RPY<double> cameraRotation = RPY<> (-25*Deg2Rad, 0, 0);
			//So the transformation matrix is created with the translation and the rotation matrices
			Transform3D<double> cameraTransfomationMatrix = Transform3D<double>(camera_position, cameraRotation.toRotation3D());
			if (debugging) cout << GREEN"	Position for Camera: " RESET << cameraTransfomationMatrix.P() << endl;
			if (debugging) cout << GREEN"	Rotation for Camera: " RESET << RPY<double>(cameraTransfomationMatrix.R()) << endl;

			//Now we can calculate the Q with inverse kinematics
			rw::invkin::JacobianIKSolver solver(device, state);
			solver.setCheckJointLimits(1); //Activate the joint limits checking
			vector<Q> qVectorSolutionIK; //It returns a vector with all the solutions
			qVectorSolutionIK = solver.solve(cameraTransfomationMatrix, state);

			/*
			 * Obtains the Path. RRT + Natural Cubic Spline
			 */
			//QPath to store the path
			QPath pathToDesiredQ;
			//We need an smart pointer for the Interpolation. I don't know why
			QPath::Ptr pathToDesiredQPtr;
			pathToDesiredQPtr = &pathToDesiredQ;

			//If there is a target Q
			if (!qVectorSolutionIK.empty()) {
				//Clear the vector of q's
				pathToDesiredQ.clear();
				//Get current position of the real robot
				client_getJointConfig.call(srv_getJointConfig);
				for (unsigned char joint = 0; joint < 7; ++joint){
					Q_current[joint] = srv_getJointConfig.response.positions[joint]*Deg2Rad;
				}
				device->setQ(Q_current, state);

				//Calculate the path from the current Q to the desired Q and show the calculation time
				if (debugging) cout << YELLOW "    Planning from " << Q_current << " to " << qVectorSolutionIK[0] << ":" RESET << endl;
				planner->query(Q_current, qVectorSolutionIK[0], pathToDesiredQ);
				if (debugging) cout << CYAN "	Solution found with " << qVectorSolutionIK.size() << " Q's" << endl;

				//For the solution, we interpolate all the Q to get an smoother path
				if (pathToDesiredQ.size()>=2){
					InterpolatorTrajectory<Q>::Ptr pathInterpolated = CubicSplineFactory::makeNaturalSpline(pathToDesiredQPtr, spline_path_dt);
					vector<Q> qVector =  pathInterpolated->getPath((double)(step_dt), false);
					for (auto q : qVector)
						sendRobotToQ(q, client_setJointConfig, srv_setJointConfig, debugging);
					pathNotFoundTimer.resetAndResume();
				}
				else {
					if (debugging) cout << MAGENTA "Not solution found. Reset in: " << pathNotFoundTimer.getTimeSec() << "/" << pathNotFoundTimeLimit << RESET<< endl << endl;
					//If we don't find a solution over the time limit, we go back to home
					if (pathNotFoundTimer.getTimeSec()>=pathNotFoundTimeLimit){
						sendRobotToQ(Q_ready, client_setJointConfig, srv_setJointConfig, debugging);
						pathNotFoundTimer.reset();
						if (debugging) ROS_WARN("Robot going home");
					}
				}
			}

			/*
			 * Publish all the messages
			 */
			if (!qVectorSolutionIK.empty()){
				//Q_desired
				path_planning::Q_desired msg_q_desired;

				for (unsigned char joint = 0; joint < 7; ++joint)
					msg_q_desired.positions[joint] = qVectorSolutionIK[0][joint];
				pub_q_desired.publish(msg_q_desired);

				//Q_real
				path_planning::Q_real msg_q_real;
				Q q_real(7);
				client_getJointConfig.call(srv_getJointConfig);
				for (unsigned char joint = 0; joint < 7; ++joint){
					msg_q_real.positions[joint] = srv_getJointConfig.response.positions[joint]*Deg2Rad;
					q_real(joint) = srv_getJointConfig.response.positions[joint]*Deg2Rad;
				}
				pub_q_real.publish(msg_q_real);


				//Camera Pose Desired
				device->setQ(qVectorSolutionIK[0], state);//Change the robot's state to the desired Q
				geometry_msgs::PoseStamped camera_pose_desired;
				Transform3D<> camera_pose_desired_transformation = Transform3D<>(wc->findFrame("Camera")->getTransform(state));
				Quaternion<> camera_pose_desired_quaternion = Quaternion<>(camera_pose_desired_transformation.R()) ;
				camera_pose_desired.header.stamp.sec = walltime.now().sec;
				camera_pose_desired.pose.position.x = camera_pose_desired_transformation.P()[0];
				camera_pose_desired.pose.position.y = camera_pose_desired_transformation.P()[1];
				camera_pose_desired.pose.position.z = camera_pose_desired_transformation.P()[2];
				camera_pose_desired.pose.orientation.x = camera_pose_desired_quaternion.getQx();
				camera_pose_desired.pose.orientation.y = camera_pose_desired_quaternion.getQy();
				camera_pose_desired.pose.orientation.z = camera_pose_desired_quaternion.getQz();
				camera_pose_desired.pose.orientation.w = camera_pose_desired_quaternion.getQw();
				pub_camera_pose_desired.publish(camera_pose_desired);


				//Camera Pose real
				geometry_msgs::PoseStamped camera_pose_real;
				device->setQ(q_real, state); //Change the robot's state to the real Q
				Transform3D<> camera_pose_real_transformation = Transform3D<>(wc->findFrame("Camera")->getTransform(state));
				Quaternion<> camera_pose_real_quaternion = Quaternion<>(camera_pose_real_transformation.R()) ;
				camera_pose_real.header.stamp.sec = walltime.now().sec;
				camera_pose_real.pose.position.x = camera_pose_real_transformation.P()[0];
				camera_pose_real.pose.position.y = camera_pose_real_transformation.P()[1];
				camera_pose_real.pose.position.z = camera_pose_real_transformation.P()[2];
				camera_pose_real.pose.orientation.x = camera_pose_real_quaternion.getQx();
				camera_pose_real.pose.orientation.y = camera_pose_real_quaternion.getQy();
				camera_pose_real.pose.orientation.z = camera_pose_real_quaternion.getQz();
				camera_pose_real.pose.orientation.w = camera_pose_real_quaternion.getQw();
				pub_camera_pose_real.publish(camera_pose_real);
			}
		}

		else {
			if (debugging) cout << MAGENTA "Not point found. Reset in " << (float)(pathNotFoundTimeLimit - pathNotFoundTimer.getTimeMs()/1000) << " seconds" << RESET<< endl << endl;
			//If we don't find a solution over the time limit, we go back to home
			if (pathNotFoundTimer.getTimeSec()>=pathNotFoundTimeLimit){
				sendRobotToQ(Q_ready, client_setJointConfig, srv_setJointConfig, debugging);
				device->setQ(Q_ready, state);
				pathNotFoundTimer.reset();
				if (debugging) ROS_WARN("Robot going home");
			}
		}

		point_updated = false;
		loop_rate.sleep();
		ros::spinOnce();
	}
}
