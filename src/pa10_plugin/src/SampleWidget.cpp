#include "SampleWidget.hpp"
#include <QFileDialog>
#include <QMessageBox>

#include <rw/invkin/AmbiguityResolver.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Math.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "pa10_plugin/getJointConfig.h"
#include "pa10_plugin/setJointConfig.h"


using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rwlibs::task;
using namespace rw::invkin;

using namespace std;


SampleWidget::SampleWidget(QWidget *parent)
: QWidget(parent)
{
	setupUi(this);
	connect(_sendJointBtn, SIGNAL(released()), this, SLOT(eventBtn()));
	connect(_getJointBtn, SIGNAL(released()), this, SLOT(eventBtn()));
}

SampleWidget::~SampleWidget()
{

}

void SampleWidget::initialize(WorkCell::Ptr workcell, rws::RobWorkStudio* rws,ros::NodeHandle* nodeHandle) {
	_rwWorkCell = workcell;
	_rws = rws;
	_state = _rws->getState();
	_nodeHandle = nodeHandle;

	_serviceClient = _nodeHandle->serviceClient<pa10_plugin::setJointConfig>("pa10/setJointsConfig");
	_serviceClientGetJoint = _nodeHandle->serviceClient<pa10_plugin::getJointConfig>("pa10/getJointConfig");

	_pointsSubscriber = _nodeHandle->subscribe("/points_server/points", 1, &SampleWidget::pointsCallback, this);

	const std::vector<rw::common::Ptr<Device> >& devices = _rwWorkCell->getDevices();
	if (devices.size() == 0)
		return;
	_device = dynamic_cast<SerialDevice*>(devices[0].get());
	std::cout <<"Loaded device " << _device->getName() << std::endl;


	if(_device->getQ(_state).size() != 7 ){
		std::cout <<"Device is not a PA10 robot" << std::endl;
		exit(0);
	}
}

void SampleWidget::stateChangedListener(const rw::kinematics::State &state)
{
	_state = state;
}


void SampleWidget::eventBtn()
{
	QObject *obj = sender();
	if(obj == _sendJointBtn){
		pa10_plugin::setJointConfig srv;
		Q q(7,0,0,0,0,0,0,0);
		if(jointconfigNumber % 2 == 0)
			q(3) = 1.57;
		else
			q(3) = 0;

		jointconfigNumber++;
		srv.request.gripper =  0;
		Q q_rq = _device->getQ(_state);
		for (uint8_t i = 0; i < 7; i++) {
			srv.request.commands[i] =  1;
			srv.request.positions[i] = q_rq(i)*Rad2Deg;
		}
		if (_serviceClient.call(srv))
		{
			ROS_INFO("Send Joints config :%f j3: %f",(double)srv.request.commands[0],q(3));
		}
		else
		{
			ROS_ERROR("Failed to call service pa10__command");
		}
	}
	else if(obj == _getJointBtn){
		pa10_plugin::getJointConfig srv;

		if (_serviceClientGetJoint.call(srv))
		{
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[0]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[1]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[2]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[3]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[4]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[5]);
			ROS_INFO("Reveved Joints config :%f",srv.response.positions[6]);
		}
		else
		{
			ROS_ERROR("Failed to call service getJoint");
		}

		Q startQ(7,srv.response.positions[0]*Deg2Rad,srv.response.positions[1]*Deg2Rad,srv.response.positions[2]*Deg2Rad,srv.response.positions[3]*Deg2Rad,srv.response.positions[4]*Deg2Rad,srv.response.positions[5]*Deg2Rad,srv.response.positions[6]*Deg2Rad);
		_device->setQ(startQ,_state);
		_rws->setState(_state);

	} 


}


bool SampleWidget::openWorkCell() {
	QMessageBox::information(this, "Open", "Open WorkCell");

	return true;
}

void SampleWidget::callback() {
	QCoreApplication::processEvents();
}

void SampleWidget::pointsCallback(const geometry_msgs::PointConstPtr & pointFromTopic){
	//Get the points
	_point.x = pointFromTopic->x;
	_point.y = pointFromTopic->y;
	_point.z = pointFromTopic->z;

	ROS_INFO("Point!");

	//Create a movable frame
	MovableFrame* markerFrame = static_cast<MovableFrame*>(_rwWorkCell->findFrame("Ball"));
	//And generates the transformation
	Transform3D<> markerTransformation = Transform3D<>(
			Vector3D<>(_point.x, _point.y, _point.z),
			RPY<>(0, 0, 0).toRotation3D());
	//So now the position is updated
	markerFrame->setTransform(markerTransformation, _state);
	//Updates the state in the RobWork Studio
	_rws->setState(_state);
}

void SampleWidget::logMotionInfo(const Q& q, const Transform3D<>& transform, unsigned int blend, int status, int turn) {
	//_log.flush();
}

void SampleWidget::logTaskInfo(QTask::Ptr task) {
	//_log.flush();
}

