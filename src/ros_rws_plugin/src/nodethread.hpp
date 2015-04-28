#ifndef NODETHREAD_HPP
#define NODETHREAD_HPP

#include <QThread>

//Robwork
#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>

//ROS
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>


class NodeThread : public QThread {
    Q_OBJECT

public:
    NodeThread();
    NodeThread(rw::models::WorkCell::Ptr workcell);
    void run() override;
    void callback(const geometry_msgs::PointStampedConstPtr & pointFromTopic);

private:
    //ROS
	ros::NodeHandle _nodeHandle;
	ros::ServiceClient _serviceClientGetJoint;
    ros::Subscriber _pointsSubscriber;
    geometry_msgs::PointStamped _point;

	rw::models::WorkCell::Ptr _workcell;
	rw::models::SerialDevice _device;
	rw::kinematics::State _state;
	rws::RobWorkStudio _rws;
};

#endif // NODETHREAD_HPP
