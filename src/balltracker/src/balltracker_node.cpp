/**
 * Receive images from an stereo camera and returns the 3D point
 * @authors Kim the master and its minions
 */

//STD
#include <vector>
#include <fstream>
#include <atomic>

//OpenCV
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>


//Namespaces
using namespace std;
using namespace cv;

//ROS Paths
#define SUBSCRIBER_CAMERA_1 "/image_raw"
#define SUBSCRIBER_CAMERA_2 "/image_raw"
#define TOPIC "/balltracker/point"
#define PARAM_VIEW_MESSAGES "/balltracker/view_messages"
#define PARAM_VIEW_IMAGES "/balltracker/view_images"
#define CAMERAS_CALIBRATION_PATH "/jijiji/calibration.txt"


//----------------------------------------------------------------
//							Classes
//----------------------------------------------------------------
/**
 * The ROS node runs in this class. Each object create its own node where two images are received from topics.
 * From this images a feature extracted and then the point from both is triangulated to obtain a 3D point.
 *
 */
class SyncedImages
{
public:
	SyncedImages()
: _nh()
, _it(_nh)
, _leftImgSub(_it, SUBSCRIBER_CAMERA_1, 1)
, _rightImgSub(_it, SUBSCRIBER_CAMERA_2, 1)
, _sync(policy_type(1), _leftImgSub, _rightImgSub)
, _updated(false)
, _view_images(false)
, _view_messages(false)
{
		//ROS Parameters
		_nh.setParam(PARAM_VIEW_MESSAGES, false);
		_nh.setParam(PARAM_VIEW_IMAGES, false);
		//Publisher
		_point_pub = _nh.advertise<geometry_msgs::Point>(TOPIC, 1);
		//Calibrate the cameras from the file
		if (!_camerasCalibrated) readStereoCameraFile(CAMERAS_CALIBRATION_PATH, _stereoCam);
		//Connect the callback
		_sync.registerCallback(boost::bind(&SyncedImages::callback, this, _1, _2));
}

	bool updated() const
	{
		return _updated;
	}

	bool images(cv::Mat &imgLeft, cv::Mat &imgRight)
	{
		if (_lock.test_and_set(memory_order_acquire))
			return false;

		_imgLeft.copyTo(imgLeft);
		_imgRight.copyTo(imgRight);
		_updated = false;
		_lock.clear(memory_order_release);
		return true;
	}

private:
	//Typedef
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> policy_type;

	//Debugging
	bool _view_messages;
	bool _view_images;

	//ROS
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;
	image_transport::SubscriberFilter _leftImgSub;
	image_transport::SubscriberFilter _rightImgSub;
	message_filters::Synchronizer<policy_type> _sync;
	ros::Publisher _point_pub;

	//Image Sync
	bool _updated;
	atomic_flag _lock{ATOMIC_FLAG_INIT};

	/*
	 * 3D point calculation
	 */
	/**
	 * Struct that defines a camera
	 */
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
			cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
					<< distortion << "\n" << transformation << "\n" << projection
					<< endl;
		}
	};

	/**
	 * Struct to contain the information of two cameras
	 */
	struct StereoPair {
		Camera cam1;
		Camera cam2;
	};

	bool _camerasCalibrated = false;

	cv::Mat _imgLeft;
	cv::Mat _imgRight;

	StereoPair _stereoCam;

	void loadCamFromStream(istream & input, Camera &cam);
	void readStereoCameraFile(const string & fileNameP, StereoPair &stereoPair);
	geometry_msgs::Point stereopsis(cv::Point & tracked_point);
	geometry_msgs::Point triangulationOpenCV(cv::Point & tracked_point_left, cv::Point & tracked_point_right);

	/*
	 * Feature Extraction
	 */
	cv::Point findFeatureBasedOnLine(Mat & image, Mat & epipolar);

	/**
	 * Callback that read the ROS images messages, transform them to OpenCV images and
	 * call the feature extraction and triangulation functions
	 * @param msgLeft
	 * @param msgRight
	 */
	void callback(sensor_msgs::ImageConstPtr const &msgLeft, sensor_msgs::ImageConstPtr const &msgRight)
	{
		//Check if the creator wants to know that the hell is going on
		ros::param::get(PARAM_VIEW_MESSAGES, _view_messages);
		ros::param::get(PARAM_VIEW_IMAGES, _view_images);

		// If currently locked, we disregard this update.
		if (_lock.test_and_set(memory_order_acquire))
			return;

		//Receive images
		_imgLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8)->image;
		_imgRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8)->image;

		//Tracking
		cv::Point tracked_point_left;
		cv::Point tracked_point_right;
		//tracked_point_left = findBall(_imgLeft);
		//tracked_point_right = findBall(_imgRight);


		//Calculate 3D
		geometry_msgs::Point point3d;
		//Stereopsis
		point3d = stereopsis(tracked_point_left);
		//Matching
		//point3d = matching(tracked_point_left, tracked_point_right);

		//Publish
		_point_pub.publish(point3d);

		//Sync
		_updated = true;
		_lock.clear(memory_order_release);
	}
};


//----------------------------------------------------------------
//						Feature extraction
//----------------------------------------------------------------
/**
 * From an image given, finds the ball and return its center
 * @param img The image to search in
 * @return The center of the ball
 */
cv::Point findBall(cv::Mat const &img)
{
	cv::Mat gray;
	cv::cvtColor(img, gray, CV_BGR2GRAY);

	// Smooth image
	cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

	// Find circles using Hough transform
	vector<cv::Vec3f> circles;
	cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 2, (gray.rows + gray.cols) / 8, 200, 200);

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

/**
 * Find the center of the tracked image based on the epipolar of the other image
 * @param image The image to obtain the feature from
 * @param epipolar The epipolar from the other image
 * @return The center of the tracked object
 */
cv::Point SyncedImages::findFeatureBasedOnLine(Mat & image, Mat & epipolar){
	cv::Point point_center;

	/*
	 * To be implemented
	 */

	return point_center;
}



//----------------------------------------------------------------
//						Triangulation
//----------------------------------------------------------------
/**
 * Reads a calibration file with the calibration parameters of a camera.
 * These are: Intrinsic, Distortion, Transformation, Projection parameters from the camera
 * and Height and Width from the image
 * @param input The file
 * @param cam The camera to store the parameters in
 */
void SyncedImages::loadCamFromStream(istream & input, Camera &cam) {
	Mat intrinsic = Mat::zeros(3, 3, CV_64F);
	Mat distortion = Mat::zeros(4, 1, CV_64F);
	Mat projection = Mat::zeros(4, 4, CV_64F);
	Mat transformation = Mat::eye(4, 4, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	Mat rotation = Mat::zeros(3, 3, CV_64F);
	double image_width, image_height;

	input.precision(20);
	input >> image_width >> image_height;

	input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
					>> intrinsic.at<double>(0, 2);
	input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
					>> intrinsic.at<double>(1, 2);
	input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
					>> intrinsic.at<double>(2, 2);

	input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
					>> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

	input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
					>> rotation.at<double>(0, 2);
	input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
					>> rotation.at<double>(1, 2);
	input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
					>> rotation.at<double>(2, 2);
	input >> translation.at<double>(0) >> translation.at<double>(1)
					>> translation.at<double>(2);

	hconcat(rotation, translation, transformation);
	Mat row = Mat::zeros(1, 4, CV_64F);
	row.at<double>(0, 3) = 1;
	transformation.push_back(row);

	Mat tmp = intrinsic;
	Mat tmp1 = Mat::zeros(3, 1, CV_64F);
	hconcat(tmp, tmp1, tmp);
	projection = tmp * transformation;

	cam.distortion = distortion;
	cam.intrinsic = intrinsic;
	cam.projection = projection;
	cam.transformation = transformation;
	cam.image_height = image_height;
	cam.image_width = image_width;
	cam.translation = translation;
	cam.rotation = rotation;
}

/**
 * Reads a calibration file where different calibration camera's parameters
 * are stored in and put them in the camera objects
 * @param fileNameP The file where to read
 * @param stereoPair Where parameters are stored in
 * @return If the read was correctly done
 */
void SyncedImages::readStereoCameraFile(const string & fileNameP, StereoPair &stereoPair) {
	int number_of_cameras;
	Camera cam1, cam2;
	ifstream ifs(fileNameP.c_str());
	if (ifs) {
		ifs >> number_of_cameras;
		if (number_of_cameras == 2) {
			loadCamFromStream(ifs, cam1);
			loadCamFromStream(ifs, cam2);
			stereoPair.cam1 = cam1;
			stereoPair.cam2 = cam2;
		}
	_camerasCalibrated = true;
	}
	else{
		ROS_WARN("Calibration file found not found");
	}
}


/**
 * OpenCV triangulation function using Projection matrices and two corresponding points
 * @param tracked_point_left The point in the left image
 * @param tracked_point_right The point in the right image
 * @return The 3D point
 */
geometry_msgs::Point SyncedImages::triangulationOpenCV(cv::Point & tracked_point_left, cv::Point & tracked_point_right){
	//Correct format for the function
	cv::Mat pnts3D(1, 1, CV_64FC4);
	cv::Mat cam0pnts(1, 1, CV_64FC2);
	cv::Mat cam1pnts(1, 1, CV_64FC2);
	cam0pnts.at<Vec2d>(0)[0] = tracked_point_left.x;
	cam0pnts.at<Vec2d>(0)[1] = tracked_point_left.y;
	cam1pnts.at<Vec2d>(0)[0] = tracked_point_right.x;
	cam1pnts.at<Vec2d>(0)[1] = tracked_point_right.y;
	//Triangulate
	cv::triangulatePoints(_stereoCam.cam1.projection, _stereoCam.cam2.projection,
			cam0pnts, cam1pnts, pnts3D);
	//Some message for make life easier
	if (_view_messages) cout << endl << "OpenCV triangulation" << endl << "Image points: " << cam0pnts << "\t"
			<< cam1pnts << endl << "Triangulated point: " << pnts3D << endl;
	if (_view_messages) cout << "Normalized: " << pnts3D / pnts3D.at<double>(3, 0) << endl;

	//Normalize the point
	geometry_msgs::Point point_geo_msg;
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
geometry_msgs::Point SyncedImages::stereopsis(cv::Point & tracked_point){
	//Copy the images
	Mat img_1, img_2;
	img_1 = _imgLeft;
	img_2 = _imgRight;

	if (_view_messages) {
		cout << "ProjectionMatrixLeft:" << endl <<_stereoCam.cam1.projection<<endl << endl;
		cout << "ProjectionMatrixRight:" << endl <<_stereoCam.cam2.projection<<endl;
	}

	//Take out the translation and rotation part from the projection matrices
	Mat Pxr = _stereoCam.cam2.projection(Range(0, 3), Range(0, 3));
	Mat pxr = _stereoCam.cam2.projection(Range(0, 3), Range(3, 4));

	Mat Pxl = _stereoCam.cam1.projection(Range(0, 3), Range(0, 3));
	Mat pxl = _stereoCam.cam1.projection(Range(0, 3), Range(3, 4));


	//Compute the optical centers. For convenience we use the inv(DECOMP_SVD)
	Mat Cr = -1.0 * Pxr.inv(DECOMP_SVD) * pxr;
	Mat tmpone = Mat::ones(1, 1, CV_64F);
	Cr.push_back(tmpone);
	Mat Cl = -1.0 * Pxl.inv(DECOMP_SVD) * pxl;
	Cl.push_back(tmpone);

	if (_view_messages) cout << "\nOptical centerLeft: " << Cl << "\nOptical centerRight" << Cr << endl;

	//Compute the epipoles
	Mat el = _stereoCam.cam1.projection * Cr;
	Mat er = _stereoCam.cam2.projection * Cl;
	if (_view_messages) cout << endl << "Epipoles:" << endl << el << endl << er << endl;

	//Create symmetric skew matrix from right epipole
	Mat erx = Mat::zeros(3, 3, CV_64F);
	erx.at<double>(0, 1) = -er.at<double>(2);
	erx.at<double>(0, 2) = er.at<double>(1);
	erx.at<double>(1, 0) = er.at<double>(2);
	erx.at<double>(1, 2) = -er.at<double>(0);
	erx.at<double>(2, 0) = -er.at<double>(1);
	erx.at<double>(2, 1) = er.at<double>(0);
	if (_view_messages) cout << endl << "Symmetric skew matrix:" << endl << erx << endl << endl;

	//Compute fundamental matrix left to right
	Mat Flr = erx * _stereoCam.cam2.projection * _stereoCam.cam1.projection.inv(DECOMP_SVD);

	if (_view_messages) cout << endl << "Fundamental matrix left to right:" << endl << Flr << endl << endl;

	vector<cv::Vec3f> lines2;
	/*
	 *
	 *
	 * Above here should be only calculated once...
	 *
	 *
	 */
	double x_left, y_left, x_right, y_right;



	x_left = tracked_point.x;
	y_left = tracked_point.y;

	// Put the chosen point into matrix and add the a 1 to the end
	cv::Mat m1(3, 1, CV_64F);
	m1.at<double>(0, 0) = x_left;
	m1.at<double>(1, 0) = y_left;
	m1.at<double>(2, 0) = 1;

	//Compute the Point of infinity for the first image
	cv::Mat M1inf = Pxl.inv(DECOMP_SVD) * m1;
	Mat tmpZer = Mat::zeros(1, 1, CV_64F);
	M1inf.push_back(tmpZer);
	if (_view_messages) cout << "M1inf: " << M1inf << endl;

	//compute the projection of Minf_1 in image 2
	cv::Mat mr = _stereoCam.cam2.projection * M1inf;
	//Normalize to image plane
	cv::Mat mrN = mr / mr.at<double>(2, 0);

	if (_view_messages) cout << "Projection of point of infinity from image 1 into image 2:" << endl << mrN << endl;

	//Normalize the epipole for image 2
	cv::Mat erN = er / er.at<double>(2, 0);

	if (_view_images) {
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
		cv::line(img_2, p1, p2, Scalar(0, 255, 0), 2, CV_AA);

		// Alternatively one can utilize fundamental matrix to compute epipolar lines,
		// here we use an OpenCV function, the epipolar line is shown in red
		vector<cv::Vec3f> lines;
		vector<cv::Point2f> points;
		points.push_back(cv::Point2f(x_left, y_left));
		cv::computeCorrespondEpilines(points, 1, Flr, lines);
		for (vector<cv::Vec3f>::const_iterator it = lines.begin();
				it != lines.end(); ++it) {
			cv::line(img_2, cv::Point(0, -(*it)[2] / (*it)[1]),
					cv::Point(img_2.cols,
							-((*it)[2] + (*it)[0] * img_2.cols) / (*it)[1]),
							cv::Scalar(0, 0, 255), 2, CV_AA	);
		}
	}

	/*
	 * Calculate Point in Image 2
	 */
	cv::Point point_right = findFeatureBasedOnLine(_imgRight, erN);
	x_right = tracked_point.x;
	y_right = tracked_point.y;

	// Put the chosen point into matrix and add the a 1 to the end
	cv::Mat m_r(3, 1, CV_64F);
	m_r.at<double>(0, 0) = x_right;
	m_r.at<double>(1, 0) = y_right;
	m_r.at<double>(2, 0) = 1;

	// Compute plucker lines parameters for image 1
	Mat mu1 = Cl(Range(0, 3), Range(0, 1)).cross(
			M1inf(Range(0, 3), Range(0, 1))) / cv::norm(M1inf);
	Mat v1 = M1inf(Range(0, 3), Range(0, 1))
						/ cv::norm(M1inf(Range(0, 3), Range(0, 1)));

	// Compute the point of infinity for the second image and compute Plucker line parameters
	cv::Mat M2inf = Pxr.inv(DECOMP_SVD) * m_r;

	if (_view_messages) cout << "Point of infinity image2: " << M2inf << endl;
	Mat mu2 = Cr(Range(0, 3), Range(0, 1)).cross(M2inf) / cv::norm(M2inf);
	Mat v2 = M2inf / cv::norm(M2inf);

	if (_view_messages) cout << "\nPlucker line parameters:\nmu1: " << mu1 << "\nv1: " << v1
			<< "\nmu2: " << mu2 << "\nv2: " << v2 << endl;

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

	cout << "\nClosest point on the two lines\nM1: " << M1 << endl;
	cout << "M2: " << M2 << endl;

	// Compute average point
	Mat avgM = M1 + (M2 - M1) / 2;
	cout << "\nAverage point: " << avgM << endl;

	geometry_msgs::Point point_geo_msg;
	point_geo_msg.x = avgM.at<double>(0,0) / avgM.at<double>(3,0);
	point_geo_msg.y = avgM.at<double>(1,0) / avgM.at<double>(3,0);
	point_geo_msg.z = avgM.at<double>(2,0) / avgM.at<double>(3,0);

	return point_geo_msg;
}


//----------------------------------------------------------------
//							Main
//----------------------------------------------------------------
/**
 * Creates a ball tracker node and maintain it alive. ALIVE!!
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "balltracker_node");
	SyncedImages si;

	//while(!si.updated())
	//; // Do nothing

	cv::Mat imgLeft, imgRight;
	si.images(imgLeft, imgRight);


	while (ros::ok()) {
		ros::spin();
	}

	return EXIT_SUCCESS;
}
