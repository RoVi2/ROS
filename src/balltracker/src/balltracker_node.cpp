#include <cmath>
#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include "syncedimages.h"

using namespace cv;
using namespace std;

#define SUB_CAMERA_LEFT_INFORMATION "/stereo_camera/left/camera_info"
#define SUB_SUB_CAMERA_RIGHT_INFORMATION "/stereo_camera/right/camera_info"
#define SUB_CAMERA_LEFT_IMAGE "/stereo_camera/left/image_raw"
#define SUB_CAMERA_RIGHT_IMAGE "/stereo_camera/right/image_raw"

#define PUB_IMAGE_LEFT "/balltracker/image/left"
#define PUB_IMAGE_RIGHT "/balltracker/image/right"
#define PUB_POINTS "/balltracker/points"

#define PARAM_FRAME_RATE "/frame_rate"
#define PARAM_DEBUGGING "/balltracker_node/debugging"
#define PARAM_VIEW_IMAGES "/balltracker_node/view_images"

//Colors
#define RESET "\e[m"
#define GREEN "\e[32m"
#define YELLOW "\e[33m"
#define MAGENTA "\e[35m"
#define CYAN "\e[36m"


/*
 * 3D Camera Definition
 */
//Struct that defines a camera
struct Camera {
	Mat intrinsic;
	Mat transformation;
	Mat distortion;
	Mat projection;
	Mat translation;
	Mat rotation;
	double image_width;
	double image_height;

	void printData() {
		cout << image_width << " " << image_height << endl << intrinsic << endl
				<< distortion << endl << transformation << endl << projection << endl;
	}
};
//Struct to contain the information of two cameras
struct StereoPair {
	Camera left;
	Camera right;
};


/**
 *	Detect and delete white areas in the image smaller than size
 * @param im
 * @param size --> adjust
 * @return image cleaned
 */
void removeSmallBlobs(Mat& im, double size )
{
	// Find all contours
	vector<vector<Point> > contours;
	blur(im, im, cv::Size(9, 9));
	findContours(im.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		// Calculate contour area
		double area = contourArea(contours[i]);
		// Remove small objects by drawing the contour with black color
		if (area > 0 && area <= size)
			drawContours(im, contours, i, CV_RGB(0,0,0), -1);
	}
}


/*
 * Global parameters. Sorry :(
 */
static StereoPair stereo_camera;
bool debugging = 1;
bool view_images = 1;

/**
 *	Find the white ball and returns its location and parameters
 * @param img
 * @param center
 * @param circumference
 * @param area
 * @return If find it or no
 */
bool findWhiteBall(cv::Mat const &img, cv::Point2d &center, double &circumference, double &area)
{
	// Make local copy to edit
	cv::Mat local_img;
	img.copyTo(local_img);

	// Smooth image to avoid lots of very small contours.
	cv::blur(local_img, local_img, cv::Size(9, 9));

	// Convert to HSV colour space.
	cv::Mat hsv_img;
	cv::cvtColor(local_img, hsv_img, CV_RGB2HSV);

	// Filter each HSV channel and combine in single binary image.
	cv::Mat filt_bin_img;

	cv::inRange(hsv_img, cv::Scalar(25, 70, 100), cv::Scalar(100, 255, 255), filt_bin_img);
	morphologyEx(filt_bin_img, filt_bin_img, 0, getStructuringElement(0, Size(9,9), Point(4,4)));
	morphologyEx(filt_bin_img, filt_bin_img, 1, getStructuringElement(0, Size(9,9), Point(4,4)));
	//removeSmallBlobs(filt_bin_img, 1000);

	// Find contours.
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(filt_bin_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	// Examine each contour. Only large ones are recognized as the ball.
	if (!contours.empty()){
		for (auto const &contour : contours) {
			double circumference_tmp = cv::arcLength(contour, true);
			double area_tmp = cv::contourArea(contour);

			double radiusThreshold = 30;

			if (circumference_tmp > 2*M_PI*radiusThreshold && area_tmp > M_PI*radiusThreshold*radiusThreshold)
			{
				cv::Moments mom = cv::moments(contour, true);
				area = area_tmp;
				circumference = circumference_tmp;
				center = cv::Point2d(mom.m10 / mom.m00, mom.m01 / mom.m00); // Mass center
				double f = (4*M_PI*area)/(pow(circumference, 2));
				if(f>0.2)
					return true;
				else 
					return false;
			}
		}
	}
	else
		center = cv::Point2d(0,0);

	return false;
}

//----------------------------------------------------------------
//						Triangulation
//----------------------------------------------------------------
/**
 * Reads a calibration file with the calibration parameters of a camera.
 * These are: Intrinsic, Distortion, Transformation, Projection parameters from the camera
 * and Height and Width from the image
 * @param cameraInfo The camera to read the parameters from
 */
void readCalibrationCameraLeft(sensor_msgs::CameraInfo cameraInfo)
{
	//if (debugging) cout << "Left camera calibration" << endl;

	Mat intrinsic = Mat::zeros(3, 3, CV_64F);
	Mat distortion = Mat::zeros(5, 1, CV_64F);
	Mat rectification = Mat::zeros(3, 3, CV_64F);
	Mat projection = Mat::zeros(3, 4, CV_64F);
	Mat transformation = Mat::eye(4, 4, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	Mat rotation = Mat::zeros(3, 3, CV_64F);
	double image_width, image_height;


	//Intrinsic
	intrinsic.at<double>(0,0) = cameraInfo.K.at(0);
	intrinsic.at<double>(0,1) = cameraInfo.K.at(1);
	intrinsic.at<double>(0,2) = cameraInfo.K.at(2);
	intrinsic.at<double>(1,0) = cameraInfo.K.at(3);
	intrinsic.at<double>(1,1) = cameraInfo.K.at(4);
	intrinsic.at<double>(1,2) = cameraInfo.K.at(5);
	intrinsic.at<double>(2,0) = cameraInfo.K.at(6);
	intrinsic.at<double>(2,1) = cameraInfo.K.at(7);
	intrinsic.at<double>(2,2) = cameraInfo.K.at(8);

	//Distortion
	distortion.at<double>(0,0) = cameraInfo.D.at(0);
	distortion.at<double>(1,0) = cameraInfo.D.at(1);
	distortion.at<double>(2,0) = cameraInfo.D.at(2);
	distortion.at<double>(3,0) = cameraInfo.D.at(3);
	distortion.at<double>(4,0) = cameraInfo.D.at(4);

	//Rectification
	rectification.at<double>(0,0) = cameraInfo.R.at(0);
	rectification.at<double>(0,1) = cameraInfo.R.at(1);
	rectification.at<double>(0,2) = cameraInfo.R.at(2);
	rectification.at<double>(1,0) = cameraInfo.R.at(3);
	rectification.at<double>(1,1) = cameraInfo.R.at(4);
	rectification.at<double>(1,2) = cameraInfo.R.at(5);
	rectification.at<double>(2,0) = cameraInfo.R.at(6);
	rectification.at<double>(2,1) = cameraInfo.R.at(7);
	rectification.at<double>(2,2) = cameraInfo.R.at(8);

	//Projection
	projection.at<double>(0,0) = cameraInfo.P.at(0);
	projection.at<double>(0,1) = cameraInfo.P.at(1);
	projection.at<double>(0,2) = cameraInfo.P.at(2);
	projection.at<double>(0,3) = cameraInfo.P.at(3);
	projection.at<double>(1,0) = cameraInfo.P.at(4);
	projection.at<double>(1,1) = cameraInfo.P.at(5);
	projection.at<double>(1,2) = cameraInfo.P.at(6);
	projection.at<double>(1,3) = cameraInfo.P.at(7);
	projection.at<double>(2,0) = cameraInfo.P.at(8);
	projection.at<double>(2,1) = cameraInfo.P.at(9);
	projection.at<double>(2,2) = cameraInfo.P.at(10);
	projection.at<double>(2,3) = cameraInfo.P.at(11);


	//Size
	image_height = cameraInfo.height;
	image_width = cameraInfo.width;

	transformation = intrinsic.inv()*projection;
	for(int i=0; i<3; i++){
		for(int j=0; j<4; j++){
			if(j!=3){
				rotation.at<double>(i,j)=transformation.at<double>(i,j);
			}
			else{
				translation.at<double>(i,0)=transformation.at<double>(i,j);
			}
		}
	}

	stereo_camera.left.distortion = distortion;
	stereo_camera.left.intrinsic = intrinsic;
	stereo_camera.left.projection = projection;
	stereo_camera.left.transformation = transformation;
	stereo_camera.left.image_height = image_height;
	stereo_camera.left.image_width = image_width;
	stereo_camera.left.translation = translation;
	stereo_camera.left.rotation = rotation;

	//if (debugging) stereo_camera.right.printData();
}

/**
 * Reads a calibration file with the calibration parameters of a camera.
 * These are: Intrinsic, Distortion, Transformation, Projection parameters from the camera
 * and Height and Width from the image
 * @param cameraInfo The camera to read the parameters from
 */
void readCalibrationCameraRight(sensor_msgs::CameraInfo cameraInfo)
{
	//if (debugging) cout << "Right camera calibration" << endl;

	Mat intrinsic = Mat::zeros(3, 3, CV_64F);
	Mat distortion = Mat::zeros(5, 1, CV_64F);
	Mat rectification = Mat::zeros(3, 3, CV_64F);
	Mat projection = Mat::zeros(3, 4, CV_64F);
	Mat transformation = Mat::eye(4, 4, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	Mat rotation = Mat::zeros(3, 3, CV_64F);
	double image_width, image_height;

	//Intrinsic
	intrinsic.at<double>(0,0) = cameraInfo.K.at(0);
	intrinsic.at<double>(0,1) = cameraInfo.K.at(1);
	intrinsic.at<double>(0,2) = cameraInfo.K.at(2);
	intrinsic.at<double>(1,0) = cameraInfo.K.at(3);
	intrinsic.at<double>(1,1) = cameraInfo.K.at(4);
	intrinsic.at<double>(1,2) = cameraInfo.K.at(5);
	intrinsic.at<double>(2,0) = cameraInfo.K.at(6);
	intrinsic.at<double>(2,1) = cameraInfo.K.at(7);
	intrinsic.at<double>(2,2) = cameraInfo.K.at(8);

	//Distortion
	distortion.at<double>(0,0) = cameraInfo.D.at(0);
	distortion.at<double>(1,0) = cameraInfo.D.at(1);
	distortion.at<double>(2,0) = cameraInfo.D.at(2);
	distortion.at<double>(3,0) = cameraInfo.D.at(3);
	distortion.at<double>(4,0) = cameraInfo.D.at(4);

	//Rectification
	rectification.at<double>(0,0) = cameraInfo.R.at(0);
	rectification.at<double>(0,1) = cameraInfo.R.at(1);
	rectification.at<double>(0,2) = cameraInfo.R.at(2);
	rectification.at<double>(1,0) = cameraInfo.R.at(3);
	rectification.at<double>(1,1) = cameraInfo.R.at(4);
	rectification.at<double>(1,2) = cameraInfo.R.at(5);
	rectification.at<double>(2,0) = cameraInfo.R.at(6);
	rectification.at<double>(2,1) = cameraInfo.R.at(7);
	rectification.at<double>(2,2) = cameraInfo.R.at(8);

	//Projection
	projection.at<double>(0,0) = cameraInfo.P.at(0);
	projection.at<double>(0,1) = cameraInfo.P.at(1);
	projection.at<double>(0,2) = cameraInfo.P.at(2);
	projection.at<double>(0,3) = cameraInfo.P.at(3);
	projection.at<double>(1,0) = cameraInfo.P.at(4);
	projection.at<double>(1,1) = cameraInfo.P.at(5);
	projection.at<double>(1,2) = cameraInfo.P.at(6);
	projection.at<double>(1,3) = cameraInfo.P.at(7);
	projection.at<double>(2,0) = cameraInfo.P.at(8);
	projection.at<double>(2,1) = cameraInfo.P.at(9);
	projection.at<double>(2,2) = cameraInfo.P.at(10);
	projection.at<double>(2,3) = cameraInfo.P.at(11);


	//Size
	image_height = cameraInfo.height;
	image_width = cameraInfo.width;

	transformation = intrinsic.inv()*projection;
	for(int i=0; i<3; i++){
		for(int j=0; j<4; j++){
			if(j!=3){
				rotation.at<double>(i,j)=transformation.at<double>(i,j);
			}
			else{
				translation.at<double>(i,0)=transformation.at<double>(i,j);
			}
		}
	}

	stereo_camera.right.distortion = distortion;
	stereo_camera.right.intrinsic = intrinsic;
	stereo_camera.right.projection = projection;
	stereo_camera.right.transformation = transformation;
	stereo_camera.right.image_height = image_height;
	stereo_camera.right.image_width = image_width;
	stereo_camera.right.translation = translation;
	stereo_camera.right.rotation = rotation;

	//if (debugging) stereo_camera.right.printData();
}

/**
 * OpenCV triangulation function using Projection matrices and two corresponding points
 * @param tracked_point_left The point in the left image
 * @param tracked_point_right The point in the right image
 * @return The 3D point
 */
geometry_msgs::Point triangulationOpenCV(cv::Point2d & tracked_point_left, cv::Point2d & tracked_point_right)
{
	//Correct format for the function
	cv::Mat pnts3D(1, 1, CV_64FC4);
	cv::Mat cam0pnts(1, 1, CV_64FC2);
	cv::Mat cam1pnts(1, 1, CV_64FC2);

	//Initialize values
	geometry_msgs::Point point_geo_msg;
	point_geo_msg.x = 0;
	point_geo_msg.y = 0;
	point_geo_msg.z = 0;
	cam0pnts.empty();
	cam1pnts.empty();
	pnts3D.empty();


	cam0pnts.at<Vec2d>(0)[0] = tracked_point_left.x;
	cam0pnts.at<Vec2d>(0)[1] = tracked_point_left.y;
	cam1pnts.at<Vec2d>(0)[0] = tracked_point_right.x;
	cam1pnts.at<Vec2d>(0)[1] = tracked_point_right.y;
	//Triangulate
	//Check if there is a valid point and triangulate
	if (!(tracked_point_left.x==0 && tracked_point_left.y==0) &&
			!(tracked_point_right.x==0 && tracked_point_right.y==0) ){
		cv::triangulatePoints(stereo_camera.left.projection, stereo_camera.right.projection,
				cam0pnts, cam1pnts, pnts3D);
		//Show the results
		if (debugging) cout << GREEN << "Left[" <<
				tracked_point_left.x << ", " << tracked_point_left.y << "] " << MAGENTA << "Right[" <<
				tracked_point_right.x << ", " << tracked_point_right.y << "] " << CYAN << "Triangulated[" <<
				pnts3D.at<double>(0,0) / pnts3D.at<double>(3,0) << ", " <<
				pnts3D.at<double>(1,0) / pnts3D.at<double>(3,0) << ", " <<
				pnts3D.at<double>(2,0) / pnts3D.at<double>(3,0) << "]" <<
				RESET << endl;
	}
	else{
		cout << YELLOW << "Lost!" << endl;
	}

	//Normalize the point

	point_geo_msg.x = pnts3D.at<double>(0,0) / pnts3D.at<double>(3,0);
	point_geo_msg.y = pnts3D.at<double>(1,0) / pnts3D.at<double>(3,0);
	point_geo_msg.z = pnts3D.at<double>(2,0) / pnts3D.at<double>(3,0);

	return point_geo_msg;
}

/**
 * Calculate the 3D point based on steropsis
 * @param tracked_point The point of one of the images
 * @return The 3D point
 */
geometry_msgs::Point stereopsis(cv::Point & tracked_point, Mat imageLeft, Mat imageRight)
{
	if (debugging) {
		cout << "ProjectionMatrixLeft:" << endl <<stereo_camera.left.projection << endl << endl;
		cout << "ProjectionMatrixRight:" << endl <<stereo_camera.right.projection << endl;
	}

	//Take out the translation and rotation part from the projection matrices
	Mat Pxr = stereo_camera.right.projection(Range(0, 3), Range(0, 3));
	Mat pxr = stereo_camera.right.projection(Range(0, 3), Range(3, 4));

	Mat Pxl = stereo_camera.left.projection(Range(0, 3), Range(0, 3));
	Mat pxl = stereo_camera.left.projection(Range(0, 3), Range(3, 4));


	//Compute the optical centers. For convenience we use the inv(DECOMP_SVD)
	Mat Cr = -1.0 * Pxr.inv(DECOMP_SVD) * pxr;
	Mat tmpone = Mat::ones(1, 1, CV_64F);
	Cr.push_back(tmpone);
	Mat Cl = -1.0 * Pxl.inv(DECOMP_SVD) * pxl;
	Cl.push_back(tmpone);

	if (debugging) cout << endl << "Optical centerLeft: " << Cl << endl << "Optical centerRight" << Cr << endl;

	//Compute the epipoles
	Mat el = stereo_camera.left.projection * Cr;
	Mat er = stereo_camera.right.projection * Cl;
	if (debugging) cout << endl << "Epipoles:" << endl << el << endl << er << endl;

	//Create symmetric skew matrix from right epipole
	Mat erx = Mat::zeros(3, 3, CV_64F);
	erx.at<double>(0, 1) = -er.at<double>(2);
	erx.at<double>(0, 2) = er.at<double>(1);
	erx.at<double>(1, 0) = er.at<double>(2);
	erx.at<double>(1, 2) = -er.at<double>(0);
	erx.at<double>(2, 0) = -er.at<double>(1);
	erx.at<double>(2, 1) = er.at<double>(0);
	if (debugging) cout << endl << "Symmetric skew matrix:" << endl << erx << endl << endl;

	//Compute fundamental matrix left to right
	Mat Flr = erx * stereo_camera.right.projection * stereo_camera.left.projection.inv(DECOMP_SVD);

	if (debugging) cout << endl << "Fundamental matrix left to right:" << endl << Flr << endl << endl;

	vector<cv::Vec3f> lines2;
	/*
	 *
	 *
	 * Above here should be only calculated once...
	 *
	 *
	 */

	// Put the chosen point into matrix and add the a 1 to the end
	cv::Mat m1(3, 1, CV_64F);
	m1.at<double>(0, 0) = tracked_point.x;
	m1.at<double>(1, 0) = tracked_point.y;
	m1.at<double>(2, 0) = 1;

	//Compute the Point of infinity for the first image
	cv::Mat M1inf = Pxl.inv(DECOMP_SVD) * m1;
	Mat tmpZer = Mat::zeros(1, 1, CV_64F);
	M1inf.push_back(tmpZer);
	if (debugging) cout << "M1inf: " << M1inf << endl;

	//compute the projection of Minf_1 in image 2
	cv::Mat mr = stereo_camera.right.projection * M1inf;
	//Normalize to image plane
	cv::Mat mrN = mr / mr.at<double>(2, 0);

	if (debugging) cout << "Projection of point of infinity from image 1 into image 2:" << endl << mrN << endl;

	//Normalize the epipole for image 2
	cv::Mat erN = er / er.at<double>(2, 0);

	if (view_images) {
		//Check if epipole is in the image else we use the line equation to find the point of intersection
		//with the image plane, workaround of the the close to zero division we end up in, when using rectified images
		cv::Point p1, p2;
		if (mrN.at<double>(0, 0) > 0 && mrN.at<double>(0, 0) < 640) {
			p1.x = erN.at<double>(0, 0);
			p1.y = erN.at<double>(1, 0);
		} else {
			double deltaA = (erN.at<double>(1, 0) - mrN.at<double>(1, 0))
																											/ (erN.at<double>(0, 0) - mrN.at<double>(0, 0));
			double b = mrN.at<double>(1, 0) - deltaA * mrN.at<double>(0, 0);
			p1.x = 0;
			p1.y = b;
		}

		p2.x = mr.at<double>(0, 0);
		p2.y = mr.at<double>(1, 0);

		// Draw epipolar line in image 2 from point to point, green line
		cv::line(imageRight, p1, p2, Scalar(0, 255, 0), 2, CV_AA);

		// Alternatively one can utilize fundamental matrix to compute epipolar lines,
		// here we use an OpenCV function, the epipolar line is shown in red
		vector<cv::Vec3f> lines;
		vector<cv::Point2f> points;
		points.push_back(cv::Point2f(tracked_point.x, tracked_point.y));
		cv::computeCorrespondEpilines(points, 1, Flr, lines);
		for (vector<cv::Vec3f>::const_iterator it = lines.begin();
				it != lines.end(); ++it) {
			cv::line(imageRight, cv::Point(0, -(*it)[2] / (*it)[1]),
					cv::Point(imageRight.cols,
							-((*it)[2] + (*it)[0] * imageRight.cols) / (*it)[1]),
							cv::Scalar(0, 0, 255), 2, CV_AA	);
		}
	}

	/*
	 * Calculate Point in Image 2
	 */
	cv::Point point_right; /*= findFeatureBasedOnLine(imageRight, erN);*/

	// Put the chosen point into matrix and add the a 1 to the end
	cv::Mat m_r(3, 1, CV_64F);
	m_r.at<double>(0, 0) = point_right.x;
	m_r.at<double>(1, 0) = point_right.y;
	m_r.at<double>(2, 0) = 1;

	//Compute plucker lines parameters for image 1
	Mat mu1 = Cl(Range(0, 3), Range(0, 1)).cross(
			M1inf(Range(0, 3), Range(0, 1))) / cv::norm(M1inf);
	Mat v1 = M1inf(Range(0, 3), Range(0, 1))
																								/ cv::norm(M1inf(Range(0, 3), Range(0, 1)));

	//Compute the point of infinity for the second image and compute Plucker line parameters
	cv::Mat M2inf = Pxr.inv(DECOMP_SVD) * m_r;

	if (debugging) cout << "Point of infinity image2: " << M2inf << endl;
	Mat mu2 = Cr(Range(0, 3), Range(0, 1)).cross(M2inf) / cv::norm(M2inf);
	Mat v2 = M2inf / cv::norm(M2inf);

	if (debugging) cout << endl << "Plucker line parameters:" << endl << "mu1: " << mu1 << endl
			<< "v1: " << v1 << endl << "mu2: " << mu2 << endl << "v2: " << v2 << endl;

	// Some intermediate steps to utilize OpenCV matrix manipulations
	Mat v2mu2T, v2mu1T, v1mu1T, v1mu2T;
	cv::transpose(v2.cross(mu2), v2mu2T);
	cv::transpose(v2.cross(mu1), v2mu1T);
	cv::transpose(v1.cross(mu1), v1mu1T);
	cv::transpose(v1.cross(mu2), v1mu2T);
	Mat v1T, v2T;
	cv::transpose(v1, v1T);
	cv::transpose(v2, v2T);

	//Compute the closest point of intersection for the two lines of infinity
	Mat M1 = (v1 * v2mu2T - (v1 * v2T) * v1 * v2mu1T)
																								/ pow(cv::norm(v1.cross(v2)), 2) * v1 + v1.cross(mu1);
	Mat M2 = (v2 * v1mu1T - (v2 * v1T) * v2 * v1mu2T)
																								/ pow(cv::norm(v2.cross(v1)), 2) * v2 + v2.cross(mu1);

	if (debugging) cout << endl << "Closest point on the two lines"<< endl << "M1: "
			<< M1 << endl << "M2: " << M2 << endl;

	// Compute average point
	Mat avgM = M1 + (M2 - M1) / 2;
	cout << endl << "Average point: " << avgM << endl;

	geometry_msgs::Point point_geo_msg;
	point_geo_msg.x = avgM.at<double>(0,0) / avgM.at<double>(3,0);
	point_geo_msg.y = avgM.at<double>(1,0) / avgM.at<double>(3,0);
	point_geo_msg.z = avgM.at<double>(2,0) / avgM.at<double>(3,0);


	return point_geo_msg;
}


//----------------------------------------------------------------
//							Main
//----------------------------------------------------------------

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "balltracker_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_left = it.advertise(PUB_IMAGE_LEFT, 1);
	image_transport::Publisher pub_right = it.advertise(PUB_IMAGE_RIGHT, 1);
	SyncedImages si(it, SUB_CAMERA_LEFT_IMAGE, SUB_CAMERA_RIGHT_IMAGE);

	ros::Publisher pub_points;
	pub_points = nh.advertise<geometry_msgs::PointStamped>(PUB_POINTS, 1);

	ros::Subscriber sub_camera_calibration_left = nh.subscribe(
			SUB_CAMERA_LEFT_INFORMATION,
			1,
			readCalibrationCameraLeft
	);
	ros::Subscriber sub_camera_calibration_right = nh.subscribe(
			SUB_SUB_CAMERA_RIGHT_INFORMATION,
			1,
			readCalibrationCameraRight
	);

	//Parameters
	nh.setParam(PARAM_DEBUGGING, true);
	nh.setParam(PARAM_VIEW_IMAGES, true);

	int frame_rate=5;
	nh.setParam(PARAM_FRAME_RATE, frame_rate);

	if (debugging) cout << "Balltracker started!" << endl;

	cv::Mat img_left, img_right;
	cv::Mat img_left_undistorted, img_right_undistorted;
	bool success_l, success_r;
	cv::Point2d center_l, center_r;
	double circum_l, circum_r, area_l, area_r;

	while (nh.ok()) {
		//Read parameters
		ros::param::get(PARAM_DEBUGGING, debugging);
		ros::param::get(PARAM_VIEW_IMAGES, view_images);

		//Update the frame rate from the param server
		ros::param::get(PARAM_FRAME_RATE, frame_rate);
		ros::Rate loop_rate(frame_rate);
		//If the images are sync
		if (si.updated()) {
			//Synchronization
			si.images(img_left, img_right);
			//Undistortion
			cv::undistort(img_left, img_left_undistorted, stereo_camera.left.intrinsic, stereo_camera.left.distortion);
			cv::undistort(img_right, img_right_undistorted, stereo_camera.left.intrinsic, stereo_camera.left.distortion);
			//Feature extraction
			success_l = findWhiteBall(img_left_undistorted, center_l, circum_l, area_l);
			success_r = findWhiteBall(img_right_undistorted, center_r, circum_r, area_r);

			if (success_l){
				cv::circle(img_left_undistorted, center_l, circum_l / (2 * M_PI), cv::Scalar(255, 0, 0), 3);
				cv::circle(img_left_undistorted, center_l, 5, cv::Scalar(0, 255, 0), 3);
			}
			else {
				center_l.x = 0;
				center_l.y = 0;
			}

			if (success_r){
				cv::circle(img_right_undistorted, center_r, circum_r / (2 * M_PI), cv::Scalar(255, 0, 0), 3);
				cv::circle(img_right_undistorted, center_r, 5, cv::Scalar(0, 255, 0), 3);
			}
			else {
				center_r.x = 0;
				center_r.y = 0;
			}


			sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img_left_undistorted).toImageMsg();
			sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img_right_undistorted).toImageMsg();
			if (view_images) pub_left.publish(msg_left);
			if (view_images) pub_right.publish(msg_right);

			geometry_msgs::PointStamped pointToPublish;
			if (success_l && success_r){
				pointToPublish.point = triangulationOpenCV(center_l, center_r);
				pointToPublish.header.frame_id = "Camera";
				pub_points.publish(pointToPublish);
			}

		}
		//Sleep baby, sleep
		ros::AsyncSpinner(0).start();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
