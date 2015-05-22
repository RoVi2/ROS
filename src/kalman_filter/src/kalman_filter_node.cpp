/**
 * Receive a 3D point and returns a prediction using a Kalman Filter
 */

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

//STD
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

//ROS Paths
#define SUBSCRIBER "/balltracker/points"
//#define SUBSCRIBER "/points_server/points"
#define TOPIC "/kalman_filter/points"
#define PARAM_DEBUGGING "/kalman_filter/debugging"
#define PARAM_FRAME_RATE "/frame_rate"

geometry_msgs::PointStamped point_original;
bool point_updated = false;
#define KALMAN_RESET_WINDOW 10

//Colors
#define RESET "\e[m"
#define GREEN "\e[32m"
#define YELLOW "\e[33m"
#define MAGENTA "\e[35m"
#define CYAN "\e[36m"

/**
 * Callback method to get the points from ROS
 */
void getROSPoints(const geometry_msgs::PointStampedConstPtr & point){
	point_original.header.frame_id = point->header.frame_id;
	point_original.header.stamp = point->header.stamp;
	point_original.point.x = point->point.x;
	point_original.point.y = point->point.y;
	point_original.point.z = point->point.z;
	point_updated = true;
}

int main(int argc, char **argv)
{
	ROS_INFO("Kalman Filter Started!");
	ros::init(argc, argv, "kalman_filter_node");

	//ROS
	ros::NodeHandle nh;
	ros::Subscriber point_sub;
	ros::Publisher point_pub;

	//Debugging parameter
	bool debugging = false;
	nh.setParam(PARAM_DEBUGGING, false);
	int frame_rate = 1;

	//Get a new image and store it in the globalImage
	point_sub = nh.subscribe(SUBSCRIBER, 1, getROSPoints);

	//Point to store the prediction
	geometry_msgs::PointStamped point_predicted;
	//Publisher for the point
	point_pub = nh.advertise<geometry_msgs::PointStamped>(TOPIC, 1);


	//Kalman Filter
	int stateSize = 9;
	int measSize = 3;
	int contrSize = 0;

	unsigned int type = CV_32F;
	KalmanFilter kf(stateSize, measSize, contrSize, type);

	Mat state(stateSize, 1, type);  //[x,y,z, v_x,v_y,v_z, a_x,a_y,a_z]
	Mat meas(measSize, 1, type);    //[z_x,z_y,z_z]
	//Mat procNoise(stateSize, 1, type)
	//[E_x,E_y,E_z, E_v_x,E_v_y,E_v_z, E_a_x,E_a_y,E_a_z,]

	//Transition State Matrix A
	//Note: set dT at each processing step!
	// [ 1 0 0	dT 	0  0	0.5*dT*dT 	0 			0		 ]
	// [ 0 1 0  0	dT 0 				0.5*dT*dT			 ]
	// [ 0 0 1  0	0  dT	0 			0			0.5*dT*dT]
	// [ 0 0 0  1	0  0 	0			0 			0		 ]
	// [ 0 0 0  0	1  0 	0			0 			0		 ]
	// [ 0 0 0  0	0  1 	0			0 			0		 ]
	// [ 0 0 0  0	0  0 	1			0 			0		 ]
	// [ 0 0 0  0	0  0 	0			1 			0		 ]
	// [ 0 0 0  0	0  0 	0			0 			1		 ]
	setIdentity(kf.transitionMatrix);
	setIdentity(kf.errorCovPost);

	//Measure Matrix H
	//[ 1 0 0 0 0 0 0 0 0]
	//[ 0 1 0 0 0 0 0 0 0]
	//[ 0 0 1 0 0 0 0 0 0]
	kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0)  = 1.0f;
	kf.measurementMatrix.at<float>(10) = 1.0f;
	kf.measurementMatrix.at<float>(20) = 1.0f;

	// Process Noise Covariance Matrix Q
	//  [Ex	 0	 0 	 0	 0	 0	 0	 0	 0	 ]
	//  [0	 Ey	 0 	 0	 0	 0	 0	 0	 0	 ]
	//  [0	 0	 Ez	 0	 0	 0	 0	 0	 0	 ]
	//  [0	 0	 0	Ev_x 0	 0	 0	 0	 0	 ]
	//  [0	 0	 0	 0	Ev_y 0	 0	 0	 0	 ]
	//  [0	 0	 0	 0	 0	Ev_z 0	 0	 0	 ]
	//  [0	 0	 0	 0	 0	 0	Ea_x 0	 0	 ]
	//  [0	 0	 0	 0	 0	 0	0	Ea_y 0	 ]
	//  [0	 0	 0	 0	 0	 0	0	 0	 Ea_z]
	kf.processNoiseCov.at<float>(0)  = 0.0003;
	kf.processNoiseCov.at<float>(10) = 0.0001;
	kf.processNoiseCov.at<float>(20) = 0.0009;
	kf.processNoiseCov.at<float>(30) = 0.0001;
	kf.processNoiseCov.at<float>(40) = 0.0003;
	kf.processNoiseCov.at<float>(50) = 0.0011;
	kf.processNoiseCov.at<float>(60) = 0.0011;
	kf.processNoiseCov.at<float>(70) = 0.0052;
	kf.processNoiseCov.at<float>(80) = 0.0241;

	//Measure Noise Covariance Matrix R
	//[ 1 0 0 ]
	//[ 0 1 0 ]
	//[ 0 0 1 ]
	kf.measurementNoiseCov = Mat::zeros(measSize, measSize, type);
	kf.measurementNoiseCov.at<float>(0)  = 0.0003059;
	kf.measurementNoiseCov.at<float>(1) = 0.0000698;
	kf.measurementNoiseCov.at<float>(2) = -0.000501;
	kf.measurementNoiseCov.at<float>(3)  = 0.0000698;
	kf.measurementNoiseCov.at<float>(4) = 0.000099;
	kf.measurementNoiseCov.at<float>(5) = -0.0000384;
	kf.measurementNoiseCov.at<float>(6)  = -0.000501;
	kf.measurementNoiseCov.at<float>(7) = -0.0000384;
	kf.measurementNoiseCov.at<float>(8) = 0.000904;

	//Variables to measure the time
	double ticks = 0;
	//And for know if something has been detected or not
	bool update_kalman = false;
	int notFoundCount = 0;

	while(ros::ok()){
		//Check the creator wants to know that the hell is going on
		ros::param::get(PARAM_DEBUGGING, debugging);
		//Updates the frame rate
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);
		/*
		 * 1. Predict
		 */
		//Get dT (dT = frameUpdateRate + featureExtraction Time + sendingMessage Time)
		double precTick = ticks;
		ticks = (double) getTickCount();

		double dT = (ticks - precTick) / getTickFrequency(); //seconds

		if (update_kalman)
		{
			//Transition Matrix
			kf.transitionMatrix.at<float>(3) = dT;
			kf.transitionMatrix.at<float>(13) = dT;
			kf.transitionMatrix.at<float>(23) = dT;
			kf.transitionMatrix.at<float>(6) = 0.5*dT*dT;
			kf.transitionMatrix.at<float>(16) = 0.5*dT*dT;
			kf.transitionMatrix.at<float>(26) = 0.5*dT*dT;

			if (debugging) cout << "dT: " << dT << endl;

			//Predict!
			state = kf.predict();

			//Publish the point
			point_predicted.point.x = state.at<float>(0);
			point_predicted.point.y = state.at<float>(1);
			point_predicted.point.z = state.at<float>(2);
			point_predicted.header.frame_id = point_original.header.frame_id;
			point_predicted.header.stamp = point_original.header.stamp;

			cout << endl << "[ " << MAGENTA << point_predicted.point.x << ", "
					<< CYAN <<  point_predicted.point.y << ", "
					<< GREEN <<  point_predicted.point.z << RESET << "]";

			point_pub.publish(point_predicted);

			if (debugging) cout << "State post:" << endl << state << endl;
		}

		/*
		 * 2. Get measurements
		 */
		geometry_msgs::PointStamped point_measured = point_original;

		/*
		 * 3. Update the Kalman Filter
		 */
		//Check if the point is bad

		//If we got a new point
		if (point_updated) {
			//Reset the counter
			notFoundCount = 0;
			//Copy the new measurements
			meas.at<float>(0) = point_measured.point.x;
			meas.at<float>(1) = point_measured.point.y;
			meas.at<float>(2) = point_measured.point.z;
			//If we are inside of the KALMAN_RESET_WINDOW
			if (update_kalman) {
				kf.correct(meas); //Kalman Correction
			}
			//Else, it is the first detection
			else {
				//Reset Error Covariance Prediction Matrix
				setIdentity(kf.errorCovPre, 1);
				//Update the state with the measures
				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);
				state.at<float>(2) = meas.at<float>(2);
				state.at<float>(3) = 0;
				state.at<float>(4) = 0;
				state.at<float>(5) = 0;
				state.at<float>(6) = 0;
				state.at<float>(7) = 0;
				state.at<float>(8) = 0;

				update_kalman = true;
			}
			if (debugging) cout << "Measure matrix:" << endl << meas << endl;
		}

		//Otherwise, if we don't have a new point
		else {
			//This counts the number of frames we have lost the object
			notFoundCount++;
			//Despite we have lost the object, if the number of times is less than the kalman
			//reset window, we keep updating the filter.
			if (notFoundCount <= KALMAN_RESET_WINDOW){
				cout << YELLOW << "Lost for " << notFoundCount
						<< "/"<< KALMAN_RESET_WINDOW << " times"
						<< RESET << endl;
				kf.statePost = state;
			}
			//If the number of times is bigger than the limit, then we have lost the object
			// and we have to restart the filter
			else {
				cout << MAGENTA << "Waiting for the object" << RESET << endl;
				update_kalman = false;
			}
		}

		point_updated = false;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
