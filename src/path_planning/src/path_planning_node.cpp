/*
 * Path Planning Node
 *
 *  Created on: Mar 28, 2015
 *      Author: Jorge Rodriguez Marin
 */

//ROBWORK
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

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
#define WORKCELL_PATH "/mnt/Free/Drive/Programming/ROS/RoVi/res/PA10Scene/ScenePA10RoVi2Demo.wc.xml"

//ROS Paths
#define SUBSCRIBER "/kalman_filter/points"
#define TOPIC "/path_planning/Q"
#define PARAM_DEBUGGING "/path_planning/debugging"

geometry_msgs::Point point_kalman;

/**
 * Callback method to get the points from ROS
 */
void callBack(const geometry_msgs::PointConstPtr & point_ros){
	point_kalman.x = point_ros->x;
	point_kalman.y = point_ros->y;
	point_kalman.z = point_ros->z;
}

int main(int argc, char** argv)
{
	ROS_INFO("Path Planning Started!");
	ros::init(argc, argv, "path_planning_node");
	//ROS
	ros::NodeHandle nh;
	ros::Subscriber point_sub;
	ros::Publisher Q_pub;

	//Debugging parameter
	bool debugging = false;
	nh.setParam(PARAM_DEBUGGING, false);

	//Get the new predicted point and store it
	point_sub = nh.subscribe(SUBSCRIBER, 1,  callBack);


	//Load the workcell and the device
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(WORKCELL_PATH);
	Device::Ptr device = wc->findDevice("ScenePA10");

	if (device == NULL) {
		cerr << "Device not found!" << endl;
		return 0;
	}



}

/*

*
 * @authors Lukas Surovchick & Jorge Rodriguez
 * @brief Reads a RobWork scene, and generates a LUA Script with the robot's paths to move a bottle from a place to another, both given previously.

#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

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

//Defines
#define MAXTIME 60

//Declarations
int path_planning(void);
void CreateFile(void);
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);




*
 * @brief First creates a LUA Script with all the information excepts the paths, then calculates the paths and writes them in the script.

int main(void) {
	CreateFile(); //Creates the LUA Script without the paths
	path_planning(); //Calculates the paths and includes them in the LUA file
	return 0;
}



*
 * @brief Creates the file and writes everything in the script except the paths

void CreateFile()
{
	ofstream myfile;
	myfile.open ("LUA.txt");
	if (myfile.is_open())
	{
		myfile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
		myfile << "state = wc:getDefaultState()" << endl;
		myfile << "device = wc:findDevice(\"KukaKr16\")" << endl;
		myfile << "gripper = wc:findFrame(\"ToolMount\");" << endl;
		myfile << "object = wc:findFrame(\"Bottle\");" << endl;
		myfile << "table = wc:findFrame(\"Table\");" << endl << endl;

		myfile << "function setQ(q)" << endl;
		myfile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
		myfile << "device:setQ(qq,state)" << endl;
		myfile << "rws.getRobWorkStudio():setState(state)" << endl;
		myfile << "rw.sleep(0.001)" << endl;
		myfile << "end" << endl << endl;

		myfile << "function attach(obj, tool)" << endl;
		myfile << "rw.gripFrame(obj, tool,state)" << endl;
		myfile << "rws.getRobWorkStudio():setState(state)" << endl;
		myfile << "rw.sleep(0.1)" << endl;
		myfile << "end " << endl << endl;

		myfile.close();
	}
	else
		cout<<"No file" << endl;
}



*
 * @brief //Calculates the paths to take the bottle and leave it in another place, and includes them in the LUA file
 * The functions start reading the scene and the elements.
 *
 * Then, given a initial point, the position of the bottle and the final place, the function generates the paths calculating statistics
 * and performance. Throughout, change the states of the robot depending if is taking or not the bottle.
 *
 * At the end, writes the paths in to the LUA Script in the appropriate format.
 *
 * @return Can return terminal errors

int path_planning(void){
	const string wcFile = "/home/veimox/Dropbox/Programming/RobWork_2/Kr16WallWorkCell/Scene.wc.xml"; //Reads the file
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	Frame* bottle = wc->findFrame("Bottle");
	Frame* gripper = wc->findFrame("ToolMount");

	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	if (bottle == NULL) {
		cerr << "Bottle not found!" << endl;
		return 0;
	}
	if (gripper == NULL) {
		cerr << "ToolMount not found!" << endl;
		return 0;
	}



	//Q!!
	Q defstate(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Q bottle_pos(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
	Q place_pos(6,1.571,0.006,0.030,0.153,0.762,4.490);

	//Generate randomness
	Math::seed();



	*
	 * Planner from defstate to Bottle_pos
	 *
	const State state = wc->getDefaultState();

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	//Lets check first if the goal is colliding
	if (!checkCollisions(device, state, detector, bottle_pos)) return 0;

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 0.1; //Distance to search the next point
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	//Calculates the path
	QPath path_defstate_to_bottle;
	cout << "Planning from " << defstate << " to " << bottle_pos << endl;
	Timer t;
	t.resetAndResume();
	//planner->query(defstate, bottle_pos, path_defstate_to_bottle, MAXTIME); //get from DEFAULT to PICK POSITION
	t.pause();

	//Shows statistics
	cout << "Path of length " << path_defstate_to_bottle.size() << " found in " << t.getTime() << " seconds." << endl << endl;
	if (t.getTime() >= MAXTIME) {cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl << endl;}



	*
	 * Planner from Bottle to Place position
	 * As we are taking the bottle we have to create a new state, thus a device too
	 *
	State withBottleState = state; //New state with the bottle
	device->setQ(bottle_pos, withBottleState); // Set device
	Kinematics::gripFrame(bottle, gripper, withBottleState); //Align the bottle to the robot

	//First, check if the goal is colliding
	if (!checkCollisions(device, withBottleState, detector, place_pos)) return 0;

	constraint = PlannerConstraint::make(&detector, device, withBottleState);
	planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect); //No need to generate a new sampler, metric, extend or change the RTTalgorithm


	//Calculates the path
	QPath path_bottle_to_placePos; //Create the path
	cout << "Planning from " << bottle_pos << " to " << place_pos << endl;
	t.resetAndResume();
	planner->query(bottle_pos, place_pos, path_bottle_to_placePos, MAXTIME); // FIND path_defstate_to_bottle FROM PICK TO PLACE POS.
	t.pause();

	//Shows statistics
	cout << "Path of length " << path_bottle_to_placePos.size() << " found in " << t.getTime() << " seconds." << endl << endl;
	if (t.getTime() >= MAXTIME) {cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl << endl;}



	*
	 * Include the paths in to the LUA Script in the appropriate format

	ofstream myfile;
	myfile.open ("LUA.txt", ios::app); //Opens the file and whatever it's written, is appended to the end of the file
	if (myfile.is_open()) {

		//The path from the bottle position to the place position
		bool attach_bottle = false;
		for (size_t pathSize=0; pathSize<path_bottle_to_placePos.size(); pathSize++) {

			myfile << "setQ({";
			for (size_t i=0; i<5; i++) myfile << path_bottle_to_placePos[pathSize][i] << ",";
			myfile << path_bottle_to_placePos[pathSize][5] << "})" << endl;

			if (attach_bottle == false) myfile << "attach(object, gripper)" << endl; //We write in the LUA file to attach the bottle just after the first path's position
			attach_bottle = true;
		}
		myfile << "attach(object, table)" << endl;

		cout << "(º-º.) LUA Script generated properly (.º-º)" << endl;
		myfile.close(); //Close the file
	}
	else {cout<<"File not found or opened" << endl;}

	return 1;
}



*
 * @brief Used to check if a point is collides with something

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}
*/

*/
