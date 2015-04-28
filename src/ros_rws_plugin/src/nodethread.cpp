#include "nodethread.hpp"

//ROS
#include <ros/ros.h>

//RobWork
#include <rw/rw.hpp>

#include <rws/RobWorkStudio.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rwlibs::task;
using namespace rw::invkin;

using namespace std;

#include "rwsplugin.hpp"
#include "ui_rwsplugin.h"


#define TOPIC "/points_server/points"

NodeThread::NodeThread()
{
}

NodeThread::NodeThread(WorkCell::Ptr workcell){WorkCell::Ptr
	//Get workcell
	_workcell = workcell;

	//Get device
	_device = dynamic_cast<SerialDevice*>(_workcell->getDevices()[0].get());
	ROS_INFO("Loaded device");
}

void NodeThread::callback(const geometry_msgs::PointStampedConstPtr & pointFromTopic){
	//Get the points
	_point.point.x = pointFromTopic->point.x;
	_point.point.y = pointFromTopic->point.y;
	_point.point.z = pointFromTopic->point.z;

	ROS_INFO("Point!");

	//Create a movable frame
	MovableFrame* markerFrame = static_cast<MovableFrame*>(_workcell->findFrame("Ball"));
	//And generates the transformation
	Transform3D<> markerTransformation = Transform3D<>(
			Vector3D<>(_point.point.x, _point.point.y, _point.point.z),
			RPY<>(0, 0, 0).toRotation3D());
	//So now the position is updated
	markerFrame->setTransform(markerTransformation, _state);
	//Updates the state in the RobWork Studio
	_rws->setState(_state);
}

void NodeThread::run()
{
    int argc = 1;
    char *argv[] = {const_cast<char *>("rwsplugin"), nullptr};
    ros::init(argc, argv, "rwsplugin_node");
    _pointsSubscriber = _nodeHandle->subscribe(TOPIC, 1, &NodeThread::callback, this);


    ros::Rate loop_rate(10);
    while (ros::ok()) {


        ros::spinOnce();
        loop_rate.sleep();
    }
}

