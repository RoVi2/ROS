/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 * http://www.robot-home.it/blog/en/software/ball-tracker-con-filtro-di-kalman/
 ******************************************/

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

// STD
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
#define MIN_S_WHITE 0
#define MAX_S_WHITE 50
#define MIN_V_WHITE 200
#define MAX_V_WHITE 255
// <<<<< Color to be tracked

#define SUBSCRIBER "/stereo_camera/left/image_raw"
#define TOPIC "/tracking/points"
#define PARAM_VIEW_IMAGES "/tracking/view_images"
#define PARAM_VIEW_RESULTS "/tracking/view_results"


cv::Point findBall(cv::Mat const &img)
{
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    // Smooth image
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

    // Find circles using hough transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, (gray.rows + gray.cols) / 8, 200, 100, 0, 0);

    if (circles.empty()) {
        ROS_WARN_STREAM("Ball tracker found no balls");
        return cv::Point();
    } else if (circles.size() > 1) {
        ROS_WARN_STREAM("Ball tracker found more than 1 ball.");
        return cv::Point();
    } else {
        return cv::Point(boost::math::round(circles[0][0]), boost::math::round(circles[0][1]));
    }
}


//--------------------------------------------------
//					Methods
//--------------------------------------------------
/**
 * Recognize the ball and returns the center
 * @param frame
 * @param res
 * @return The center of the ball
 */
cv::Point ballsRecognition(Mat & frame, Mat & res){
	//Get Parameters
	bool view_images = false;
	ros::param::get(PARAM_VIEW_IMAGES, view_images);
	bool view_results = false;
	ros::param::get(PARAM_VIEW_RESULTS, view_results);

	//The vector to store the balls
	vector<Rect> ballsBox;
	cv::Point ball;

	if(!frame.empty() && !res.empty()){
		// >>>>> Noise smoothing
		Mat blur;
		GaussianBlur(frame, blur, Size(5, 5), 3.0, 3.0);
		// <<<<< Noise smoothing

		// >>>>> HSV conversion
		Mat frmHsv;
		cvtColor(blur, frmHsv, CV_BGR2HSV);
		// <<<<< HSV conversion

		// >>>>> Color Thresholding
		// Note: change parameters for different colors
		Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
		inRange(frmHsv, Scalar(0, MIN_S_WHITE, MIN_V_WHITE),
				Scalar(360, MAX_S_WHITE, MAX_V_WHITE), rangeRes);
		// <<<<< Color Thresholding

		// >>>>> Improving the result
		erode(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
		dilate(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
		// <<<<< Improving the result

		// Thresholding viewing
		if (view_images) imshow("Threshold", rangeRes);

		ball=findBall(frame);		
	}
	
	ROS_INFO("Detecting");
	return ball;
}


//--------------------------------------------------
//					Tracking Class
//--------------------------------------------------

void getROSimage(const sensor_msgs::ImageConstPtr& msg, Mat & image)
{
	cv_bridge::CvImagePtr cv_ptr;

	// Converts the image
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	image = cv_ptr->image;

};


//--------------------------------------------------
//					Main
//--------------------------------------------------
/**
 *
 */
int main(int argc, char **argv)
{
	ROS_INFO("2D Tracking Started!");

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub;
	ros::Publisher points_pub;

	// Parameters
	nh.setParam(PARAM_VIEW_IMAGES, true);
	nh.setParam(PARAM_VIEW_RESULTS, false);

	// The Mat to store the original image
	Mat imageOriginal;

	// Get a new image and store it in the globalImage
	image_sub = it.subscribe(SUBSCRIBER, 1, boost::bind(getROSimage, _1, boost::ref(imageOriginal)));

	//std_msgs::String points;
	geometry_msgs::Point point_msg;
	points_pub = nh.advertise<geometry_msgs::Point>(TOPIC, 1);


	// >>>> Kalman Filter
	int stateSize = 6;
	int measSize = 2;
	int contrSize = 0;

	unsigned int type = CV_32F;
	KalmanFilter kf(stateSize, measSize, contrSize, type);

	Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	//Mat procNoise(stateSize, 1, type)
	// [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	// Transition State Matrix A
	// Note: set dT at each processing step!
	// [ 1 0 dT 0  0.5*dT*dT 0 ]
	// [ 0 1 0  dT 0 0.5*dT*dT ]
	// [ 0 0 1  0  0 0 ]
	// [ 0 0 0  1  0 0 ]
	// [ 0 0 0  0  1 0 ]
	// [ 0 0 0  0  0 1 ]
	setIdentity(kf.transitionMatrix);

	// Measure Matrix H
	// [ 1 0 0 0 0 0 ]
	// [ 0 1 0 0 0 0 ]
	// [ 0 0 0 0 1 0 ]
	// [ 0 0 0 0 0 1 ]
	kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	//kf.measurementMatrix.at<float>(16) = 1.0f;
	//kf.measurementMatrix.at<float>(23) = 1.0f;

	// Process Noise Covariance Matrix Q
	// [ Ex 0  0    0 0    0 ]
	// [ 0  Ey 0    0 0    0 ]
	// [ 0  0  Ev_x 0 0    0 ]
	// [ 0  0  0    Ev_y 0 0 ]
	// [ 0  0  0    0 Ea_x 0 ]
	// [ 0  0  0    0 0 Ea_y ]
	//setIdentity(kf.processNoiseCov, Scalar(1e-2));
	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-2;
	kf.processNoiseCov.at<float>(14) = 2.0f;
	kf.processNoiseCov.at<float>(21) = 1.0f;
	kf.processNoiseCov.at<float>(28) = 1e-2;
	kf.processNoiseCov.at<float>(35) = 1e-2;

	// Measures Noise Covariance Matrix R
	setIdentity(kf.measurementNoiseCov, Scalar(1e-1));
	// <<<< Kalman Filter

	double ticks = 0;
	bool found = false;

	int notFoundCount = 0;


	/*
	 * Refresh frequency
	 */
	ros::Rate loop_rate(1);

	while(ros::ok()){

		//Get parameters
		bool view_images; //Show the images in new windows
		nh.getParam(PARAM_VIEW_IMAGES, view_images);

		bool view_results; //Show the results through the terminal
		nh.getParam(PARAM_VIEW_RESULTS, view_results);

		// And copy it to a Mat where we can work in
		Mat frame = imageOriginal;
		if (frame.empty()) ROS_ERROR("No image received");

		//Get dT
		double precTick = ticks;
		ticks = (double) getTickCount();

		double dT = (ticks - precTick) / getTickFrequency(); //seconds

		//Matrix to show the results
		Mat res;
		frame.copyTo(res);
		if (found)
		{
			// >>>> Matrix A
			kf.transitionMatrix.at<float>(2) = dT;
			kf.transitionMatrix.at<float>(9) = dT;
			//kf.transitionMatrix.at<float>(4) = 0.5*dT*dT;
			//kf.transitionMatrix.at<float>(11) = 0.5*dT*dT;
			// <<<< Matrix A

			if (view_results) cout << "dT: " << dT << endl;

			state = kf.predict();

			//Publish the points
			point_msg.x = state.at<float>(0);
			point_msg.y = state.at<float>(1);

			points_pub.publish(point_msg);

			//Show the prediction in the res image
			if (view_results) cout << "State post:" << endl << state << endl;

			if(view_images){
				Rect predRect;
				predRect.width = state.at<float>(4);
				predRect.height = state.at<float>(5);
				predRect.x = state.at<float>(0) - predRect.width / 2;
				predRect.y = state.at<float>(1) - predRect.height / 2;

				Point center;
				center.x = state.at<float>(0);
				center.y = state.at<float>(1);
				circle(res, center, 2, CV_RGB(255,0,0), -1);

				rectangle(res, predRect, CV_RGB(255,0,0), 2);
			}
		}

		//Ball recognition with Color > Contour > Center of masses
		cv::Point balls = ballsRecognition(frame, res);

		// >>>>> Kalman Update
		if (false)
		{
			notFoundCount++;
			if (view_results) cout << "notFoundCount:" << notFoundCount << endl;
			if(notFoundCount >= 10) found = false;
			else kf.statePost = state;
		}
		else
		{
			notFoundCount = 0;

			meas.at<float>(0) = balls.x;
			meas.at<float>(1) = balls.y;

			if (!found) // First detection!
			{
				//setIdentity(kf.errorCovPre, 1);
				kf.errorCovPre.at<float>(0) = 1; // px
				kf.errorCovPre.at<float>(7) = 1; // px
				kf.errorCovPre.at<float>(14) = 1;
				kf.errorCovPre.at<float>(21) = 1;
				kf.errorCovPre.at<float>(28) = 1; // px
				kf.errorCovPre.at<float>(35) = 1; // px

				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);

				found = true;
			}
			else
				kf.correct(meas); // Kalman Correction

			if (view_results) cout << "Measure matrix:" << endl << meas << endl;
		}
		// <<<<< Kalman Update

		// Final result
		if (!imageOriginal.empty() && view_images) imshow("Original Image", imageOriginal);
		waitKey(5);
		if (!res.empty() && view_images) imshow("Final Image", res);
		waitKey(5);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
