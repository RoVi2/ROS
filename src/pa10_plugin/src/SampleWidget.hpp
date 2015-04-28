/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef SampleWIDGET_HPP
#define SampleWIDGET_HPP


//#include <rw/common/LogFileWriter.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/FKRange.hpp>

#include <rw/invkin/PieperSolver.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

#include <rwlibs/task/Task.hpp>

#include <rws/RobWorkStudio.hpp>

#include <QWidget>
#include <QTimer>
#include "ui_SampleWidget.h"

#include <vector>
#include <utility>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

class SampleWidget : public QWidget, private Ui::SampleWidgetClass
{
	Q_OBJECT

public:
	SampleWidget(QWidget *parent = 0);
	~SampleWidget();
	void initialize(rw::models::WorkCell::Ptr workcell, rws::RobWorkStudio* rws, ros::NodeHandle* nodeHandle);

	void stateChangedListener(const rw::kinematics::State &state);
private:
	ros::NodeHandle *_nodeHandle;
	ros::ServiceClient _serviceClient;
	ros::ServiceClient _serviceClientSetQueue;
	ros::ServiceClient _serviceClientGetJoint;
    ros::Subscriber _pointsSubscriber;
    geometry_msgs::Point_<float> _point;

	private slots:

	void callback();
    void pointsCallback(const geometry_msgs::PointConstPtr & pointFromTopic);
	void eventBtn();

	private:

	//QTimer _updateTimer;

	rw::math::Transform3D<> _w2blok;
	rw::models::WorkCell::Ptr _rwWorkCell;
	rw::models::SerialDevice *_device;
	rw::kinematics::State _state;
	rw::common::PropertyMap _properties;
	rws::RobWorkStudio* _rws;
	bool openWorkCell();

	//rw::common::LogFileWriter _log;
	void logTaskInfo(rwlibs::task::QTask::Ptr task);
	int jointconfigNumber;
	void logMotionInfo(const rw::math::Q& q, const rw::math::Transform3D<>& transform, unsigned int blend, int status, int turn);
};

#endif // SampleWIDGET_HPP
