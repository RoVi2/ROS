#include "rosnodethread.h"

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include "pa10plugin.h"
#include "pa10_dummy/getJointConfig.h"

#define PARAM_FRAME_RATE "/frame_rate"
#define BALL_PREDICTED_TOPIC "/kalman_filter/points"
//#define BALL_DETECTED_TOPIC "/points_server/points"
#define BALL_DETECTED_TOPIC "/balltracker/points"
//#define SRV_GET_JOINT "pa10/getJointConfig"
#define SRV_GET_JOINT "/getJointConfig"

using namespace rw::kinematics;
using namespace rw::math;


RosNodeThread::RosNodeThread(QObject *parent)
: QThread(parent)
{
}

/**
 * Callback method to get the points from ROS
 */
rw::math::Transform3D<> ballPredictedTransformation;
void ballPredictedCallback(const geometry_msgs::PointStampedConstPtr & point_ros){
	ballPredictedTransformation = rw::math::Transform3D<>(
			rw::math::Vector3D<>(point_ros->point.x,point_ros->point.y,point_ros->point.z),
			rw::math::RPY<>(0,0,0).toRotation3D()
	);
}
rw::math::Transform3D<> ballDetectedTransformation;
rw::math::Transform3D<> ballDetectedTransformationSecond;
void ballDetectedCallback(const geometry_msgs::PointStampedConstPtr & point_ros){
	ballDetectedTransformation = ballDetectedTransformationSecond;
	ballDetectedTransformationSecond = rw::math::Transform3D<>(
			rw::math::Vector3D<>(point_ros->point.x,point_ros->point.y,point_ros->point.z),
			rw::math::RPY<>(0,0,0).toRotation3D()
	);
}

void RosNodeThread::run()
{
	stopNode_ = false;
	emit rwsLogMsg("ROS node starting...\n", rw::common::Log::LogIndex::Info);
	int argc = 1;
	char *argv[] = {const_cast<char *>("pa10_plugin"), nullptr};
	ros::init(argc, argv, "pa10_plugin_node");
	ros::NodeHandle nh;
	ros::Subscriber ball_predicted_subscriber = nh.subscribe(BALL_PREDICTED_TOPIC, 1, ballPredictedCallback);
	ros::Subscriber ball_detected_subscriber = nh.subscribe(BALL_DETECTED_TOPIC, 1, ballDetectedCallback);
	ros::ServiceClient sc_get_joint_conf = nh.serviceClient<pa10_dummy::getJointConfig>(SRV_GET_JOINT);
	pa10_dummy::getJointConfig srv;
	ros::Rate loop_rate(2);

	while (nh.ok() && !stopNode_) {
		if (!stopRobot_){
			if (sc_get_joint_conf.call(srv)) {
				std::size_t n = srv.response.positions.size();
				rw::math::Q q(n);

				for (unsigned char i = 0; i < n; ++i)
					q(i) = srv.response.positions[i]/rw::math::Rad2Deg;

				emit qUpdated(q);
			} else {
				emit rwsLogMsg("Failed call to getJointConfig service.\n", rw::common::Log::LogIndex::Info);
				ROS_WARN("Failed call to getJointConfig service.");
			}

			//Updates the ball position in Robwork
			emit ballPredictedUpdated(ballPredictedTransformation);
			emit ballDetectedUpdated(ballDetectedTransformation);

			ros::AsyncSpinner(0).start();
			loop_rate.sleep();
		}
	}

	emit rwsLogMsg("ROS node stopping...\n", rw::common::Log::LogIndex::Info);
}

void RosNodeThread::stopNode()
{
	emit rwsLogMsg("Node Stopped!\n", rw::common::Log::LogIndex::Info);
	stopNode_ = true;
}

void RosNodeThread::startStopRobot(){
	if (!stopRobot_){
		emit rwsLogMsg("Robot Stopped!\n", rw::common::Log::LogIndex::Info);
		stopRobot_ = true;
	}
	else{
		emit rwsLogMsg("Robot Started!\n", rw::common::Log::LogIndex::Info);
		stopRobot_ = false;
	}
}

void RosNodeThread::readSlider(int loopRate){
	//Due to the slider only lets integer numbers and we are interested in decimals
	loopRate_ = loopRate;
}
