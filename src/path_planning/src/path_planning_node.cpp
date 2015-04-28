/*
 * Path Planning Node
 *
 *  Created on: Mar 28, 2015
 *      Author: Jorge Rodriguez Marin
 */

//RobWork
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

//Services
#include <path_planning/getJointConfig.h>
#include <path_planning/setJointConfig.h>
#include <path_planning/addToQueue.h>
#include <path_planning/addToQueue.h>

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



//RobWork Paths
#define WORKCELL_PATH "../../res/PA10Scene/ScenePA10RoVi2Demo.wc.xml"
#define ROBOT_NAME "PA10"

//ROS Paths
#define SUBSCRIBER "/points_server/points"
#define TOPIC "/path_planning/Q"
#define PARAM_DEBUGGING "/path_planning/debugging"
#define PARAM_FRAME_RATE "/frame_rate"
#define PARAM_PATH_NOT_FOUND_TIME_LIMIT "/path_planning/path_not_found_time_limit"

//Global variables
Vector3D<double> point_kalman;

/**
 * Callback method to get the points from ROS
 */
void callBack(const geometry_msgs::PointStampedConstPtr & point_ros){
	point_kalman[0] = point_ros->point.x;
	point_kalman[1] = point_ros->point.y;
	point_kalman[2] = point_ros->point.z;
}

int main(int argc, char** argv)
{
	/*
	 * ROS Stuff
	 */
	ROS_INFO("Path Planning Started!");
	ros::init(argc, argv, "path_planning_node");
	ros::NodeHandle nh;
	ros::Subscriber point_sub;
	ros::Publisher Q_pub;

	//Debugging parameter
	bool debugging = true;
	nh.setParam(PARAM_DEBUGGING, true);
	int frame_rate = 1;

	//Get the new predicted point and store it
	point_sub = nh.subscribe(SUBSCRIBER, 1,  callBack);

	//Publisher of the Q
	//Q_pub = nh.advertise<geometry_msgs::Point>(TOPIC, 1);

	//Services
	ros::ServiceClient client_getJointConfig = nh.serviceClient<path_planning::getJointConfig>("pa10/getJointConfig");
	ros::ServiceClient client_setJointConfig = nh.serviceClient<path_planning::setJointConfig>("pa10/setJointsConfig");
	path_planning::getJointConfig srv_getJointConfig;
	path_planning::setJointConfig srv_setJointConfig;

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
	Q Q_home = device->getQ(state);
	Timer pathNotFoundTimer;
	int pathNotFoundTimeLimit = 10;
	/*
	if (client_getJointConfig.call(srv_getJointConfig)){
    	for (unsigned char joint=0; joint<device->getDOF(); joint++)
    		Q_current(joint) = srv_getJointConfig.response.positions[joint];
    	device->setQ(Q_current, state);
    }
    else ROS_WARN("Real Robot not found!");
	 */

	double extend = 0.01; //Distance to search the next point
	QPath pathToDesiredQ;
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);


	/*	cout << "From " << device->getQ(state) << " to " << Q_home << endl;
	Timer t;
	t.resetAndResume();
	planner->query(device->getQ(state), Q_home, pathToDesiredQ, 10);
	t.pause();
	cout << "Path of length " << pathToDesiredQ.size() << " found in " << t.getTimeMs() << " miliseconds." << endl << endl;

	for (auto q : pathToDesiredQ){
		for (unsigned char joint=0; joint<device->getDOF(); joint++)
			srv_setJointConfig.request.positions[joint] = q[joint];
		if (debugging) cout << q << endl;
		//if (client_setJointConfig.call(srv_setJointConfig)) ROS_INFO("Joint sent correctly!");
	}

	cout << endl << "YYAAA" << endl;*/

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

		//Generate randomness
		Math::seed();

		//Obtain from the kalman point, the point where we want to put the camera.
		//This point for now will be that which is at 0.5.m from the object and is contained
		//	in (1) a plane that with the kalman point is parallel to the ground and (2) also
		//	in the plane generated by the robot's base axis and the kalman point.

		//Translate the kalman point from the camera to the robot's base
		if (debugging) cout << "Point from: " << endl;
		if (debugging) cout << "	Camera: "<< point_kalman << endl;
		point_kalman = (device->baseTframe(wc->findFrame("Camera"), state)) * point_kalman;
		if (debugging) cout << "	Base: "<< point_kalman << endl;
		//Calculate in polar coordinates so it will simplify the calculations P(Radius, angle, height)
		double radius = sqrt(pow(point_kalman[0], 2) +  pow(point_kalman[1], 2));
		double angle = atan2(point_kalman[1],point_kalman[0]);
		double height = point_kalman[2];
		Vector3D<double> point_kalman_polar (radius, angle, height);
		if (debugging) cout << "	and in Polar: " << point_kalman_polar << endl;
		//The camera position will be 0.5 meters from the kalman point in the radial direction
		Vector3D<double> camera_position_polar = point_kalman_polar - Vector3D<double>(0.3,0,0);
		Vector3D<double> camera_position (
				camera_position_polar[0]*cos(camera_position_polar[1]), //r*cos(angle)
				camera_position_polar[0]*sin(camera_position_polar[1]), //r*sin(angle)
				camera_position_polar[2] //height
		);
		//The rotation has to be that which is oriented to the point.
		RPY<double> cameraRotation = RPY<> (-1.571, 0, -1.571);
		//So the transformation matrix is created with the translation and the rotation matrices
		Transform3D<double> cameraTransfomationMatrix = Transform3D<double>(camera_position, cameraRotation.toRotation3D());
		if (debugging) cout << "	Position for Camera: " << cameraTransfomationMatrix.P() << endl;
		if (debugging) cout << "	Rotation for Camera: " << RPY<double>(cameraTransfomationMatrix.R()) << endl;

		//Now we can calculate the Q with inverse kinematics
		rw::invkin::JacobianIKSolver solver(device, state);
		solver.setCheckJointLimits(1); //Activate the joint limits checking
		vector<Q> qVectorSolution;
		qVectorSolution = solver.solve(cameraTransfomationMatrix, state);

		//Use RRT to calculate the path from the current Q to the desired Q
		if (!qVectorSolution.empty()) {
			pathToDesiredQ.clear();
			planner->query(device->getQ(state), qVectorSolution[0], pathToDesiredQ);
			for (auto q : pathToDesiredQ){
				device->setQ(q, state);
				if (debugging) cout << q << endl;
				//Publish the Qs
			}
			pathNotFoundTimer.resetAndResume();
		}
		else {
			if (debugging) cout << "Not solution found" << endl << endl;
			//If we dont find a solution over the time limit, we go back to home
			if (pathNotFoundTimer.getTimeSec()>=pathNotFoundTimeLimit){
				device->setQ(Q_home, state);
				pathNotFoundTimer.reset();
			}
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
}
