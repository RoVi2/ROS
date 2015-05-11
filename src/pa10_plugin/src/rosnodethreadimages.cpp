#include "rosnodethreadimages.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "pa10plugin.h"
#include "pa10_dummy/getJointConfig.h"

#include <opencv2/opencv.hpp>

#define PARAM_FRAME_RATE "/frame_rate"
#define LEFT_IMAGE_TOPIC "/monocamera_camera/image"
#define RIGHT_IMAGE_TOPIC "/monocamera_camera/image"

using namespace cv;


RosNodeThreadImages::RosNodeThreadImages(QObject *parent)
: QThread(parent)
{
}

/**
 * Callback method to get the left image from ROS
 */
Mat leftImage;
void leftImageCallback(sensor_msgs::ImageConstPtr const &left){
	leftImage = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::RGB8)->image;
}
/**
 * Callback method to get the right image from ROS
 */
Mat rightImage;
void rightImageCallback(sensor_msgs::ImageConstPtr const &right){
	rightImage = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::RGB8)->image;
}

/*
 *
 */
void RosNodeThreadImages::run()
{
	stopNode_ = false;
	stopImages_ = false;
	emit rwsLogMsg("ROS node starting...\n", rw::common::Log::LogIndex::Info);
	int argc = 1;
	char *argv[] = {const_cast<char *>("pa10_plugin"), nullptr};
	ros::init(argc, argv, "pa10_plugin_node");
	ros::NodeHandle nh;
	ros::Subscriber left_image_subscriber = nh.subscribe(LEFT_IMAGE_TOPIC, 1, leftImageCallback);
	ros::Subscriber right_image_subscriber = nh.subscribe(RIGHT_IMAGE_TOPIC, 1, rightImageCallback);
	int frame_rate = 1;

	while (nh.ok() && !stopNode_) {
		if (!stopImages_){
			// Get the global frame rate
			ros::param::get(PARAM_FRAME_RATE, frame_rate);
			ros::Rate loop_rate(frame_rate);


			//Transform the OpenCV Image into RW image
			rw::sensor::Image leftImageRW;
			leftImageRW = rw::sensor::Image((char*)leftImage.data, leftImage.cols, leftImage.rows,
					rw::sensor::Image::RGB, rw::sensor::Image::Depth8U);
			rw::sensor::Image rightImageRW;
			rightImageRW = rw::sensor::Image((char*)rightImage.data, rightImage.cols, rightImage.rows,
					rw::sensor::Image::RGB, rw::sensor::Image::Depth8U);
			//Send them
			emit leftImageUpdated(leftImageRW);
			emit rightImageUpdated(rightImageRW);


			ros::spinOnce();
			loop_rate.sleep();
		}
	}


	emit rwsLogMsg("ROS node stopping...\n", rw::common::Log::LogIndex::Info);
}

void RosNodeThreadImages::stopNode()
{
	emit rwsLogMsg("Node Stopped!\n", rw::common::Log::LogIndex::Info);
	stopNode_ = true;
}

void RosNodeThreadImages::startStopImages(){
	if (!stopImages_){
		emit rwsLogMsg("Robot Stopped!\n", rw::common::Log::LogIndex::Info);
		stopImages_ = true;
	}
	else{
		emit rwsLogMsg("Robot Started!\n", rw::common::Log::LogIndex::Info);
		stopImages_ = false;
	}
}
