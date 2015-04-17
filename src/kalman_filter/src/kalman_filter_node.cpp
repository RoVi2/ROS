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
#include <geometry_msgs/Point.h>

//STD
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

//ROS Paths
#define SUBSCRIBER "/balltracker/point"
#define TOPIC "/kalman_filter/points"
#define PARAM_DEBUGGING "/kalman_filter/debugging"

geometry_msgs::Point point_original;
#define KALMAN_RESET_WINDOW 10

/**
 * Callback method to get the points from ROS
 */
void getROSPoints(const geometry_msgs::PointConstPtr & point){
	point_original.x = point->x;
	point_original.y = point->y;
	point_original.z = point->z;
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

	//Get a new image and store it in the globalImage
	point_sub = nh.subscribe(SUBSCRIBER, 1, getROSPoints);

	//Point to store the prediction
	geometry_msgs::Point point_predicted;
	//Publisher for the point
	point_pub = nh.advertise<geometry_msgs::Point>(TOPIC, 1);


	//Kalman Filter
	int stateSize = 9;
	int measSize = 3;
	int contrSize = 0;

	unsigned int type = CV_32F;
	KalmanFilter kf(stateSize, measSize, contrSize, type);

	Mat state(stateSize, 1, type);  //[x,y,<, v_x,v_y,v_z, a_x,a_y,a_z]
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
	kf.processNoiseCov.at<float>(0)  = 1e-2;
	kf.processNoiseCov.at<float>(10) = 1e-2;
	kf.processNoiseCov.at<float>(20) = 1e-2;
	kf.processNoiseCov.at<float>(30) = 2.0f;
	kf.processNoiseCov.at<float>(40) = 2.0f;
	kf.processNoiseCov.at<float>(50) = 2.0f;
	kf.processNoiseCov.at<float>(60) = 3.0f;
	kf.processNoiseCov.at<float>(70) = 3.0f;
	kf.processNoiseCov.at<float>(80) = 3.0f;

	//Measures Noise Covariance Matrix R
	setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

	//Variables to measure the time
	double ticks = 0;
	//And for know if something has been detected or not
	bool found = false;
	int notFoundCount = 0;

	/*
	 * Refresh frequency
	 */
	ros::Rate loop_rate(1);

	while(ros::ok()){
		//Check the creator wants to know that the hell is going on
		ros::param::get(PARAM_DEBUGGING, debugging);

		/*
		 * 1. Predict
		 */
		//Get dT (dT = frameUpdateRate + featureExtraction Time + sendingMessage Time)
		double precTick = ticks;
		ticks = (double) getTickCount();

		double dT = (ticks - precTick) / getTickFrequency(); //seconds

		if (found)
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

			//Publish the points
			point_predicted.x = state.at<float>(0);
			point_predicted.y = state.at<float>(1);
			point_predicted.z = state.at<float>(2);

			point_pub.publish(point_predicted);

			//Show the prediction in the res image
			if (debugging) cout << "State post:" << endl << state << endl;
		}
		else
		{
			//Reset the point
			point_predicted.x = 0;
			point_predicted.y = 0;
			point_predicted.z = 0;
			//Publish it
			point_pub.publish(point_predicted);
		}

		/*
		 * 2. Get measurements
		 */
		geometry_msgs::Point point_measured = point_original;

		/*
		 * 3. Update the Kalman Filter
		 */
		//Check if the point is bad
		if (point_original.x == 0.0 && point_original.y == 0.0 && point_original.z == 0.0)
		{
			//This counts the number of frames we have lost the object
			notFoundCount++;
			//If the number of times is bigger than the limit, then we have lost the object and we have to restart the filter
			if (notFoundCount >= KALMAN_RESET_WINDOW){
				found = false;
				if (debugging) cout << "Waiting for the object" << endl;
			}
			else if (debugging) cout << "Lost for " << notFoundCount << "/"<< KALMAN_RESET_WINDOW << " times" << endl;
			//Despite we have lost the object, if the number of times is less than the kalman
			//reset window, we keep updating the filter.
			else kf.statePost = state;
		}
		//If not, updates the filter
		else
		{
			notFoundCount = 0;

			meas.at<float>(0) = point_original.x;
			meas.at<float>(1) = point_original.y;
			meas.at<float>(2) = point_original.z;

			if (!found) //First detection!
			{
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

				found = true;
			}
			else
			{
				kf.correct(meas); //Kalman Correction
			}
			if (debugging) cout << "Measure matrix:" << endl << meas << endl;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
