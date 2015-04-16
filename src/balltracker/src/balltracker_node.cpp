#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>

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

//using namespace std;
//using namespace cv;

#include <atomic>

class SyncedImages
{
public:
    SyncedImages()
        : _nh()
        , _it(_nh)
        , _leftImgSub(_it, "/image_raw", 1)
        , _rightImgSub(_it, "/image_raw", 1)
        , _sync(policy_type(1), _leftImgSub, _rightImgSub)
        , _updated(false)
    {
        _sync.registerCallback(boost::bind(&SyncedImages::callback, this, _1, _2));
    }

    bool updated() const
    {
        return _updated;
    }

    bool images(cv::Mat &imgLeft, cv::Mat &imgRight)
    {
        if (_lock.test_and_set(std::memory_order_acquire))
            return false;

        _imgLeft.copyTo(imgLeft);
        _imgRight.copyTo(imgRight);
        _updated = false;
        _lock.clear(std::memory_order_release);
        return true;
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> policy_type;

    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::SubscriberFilter _leftImgSub;
    image_transport::SubscriberFilter _rightImgSub;
    message_filters::Synchronizer<policy_type> _sync;
    bool _updated;
    std::atomic_flag _lock{ATOMIC_FLAG_INIT};
    cv::Mat _imgLeft;
    cv::Mat _imgRight;

    void callback(sensor_msgs::ImageConstPtr const &msgLeft, sensor_msgs::ImageConstPtr const &msgRight)
    {
        // If currently locked, we disregard this update.
        if (_lock.test_and_set(std::memory_order_acquire))
            return;

        _imgLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8)->image;
        _imgRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8)->image;
        _updated = true;
        _lock.clear(std::memory_order_release);
    }
};

cv::Point findBall(cv::Mat const &img)
{
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    // Smooth image
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

    // Find circles using hough transform
    std::vector<cv::Vec3f> circles;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "balltracker_node");
    SyncedImages si;

    while(!si.updated())
        ; // Do nothing

    cv::Mat imgLeft, imgRight;
    si.images(imgLeft, imgRight);


    while (ros::ok()) {
        ros::spin();
    }

    return EXIT_SUCCESS;
}


/*
double tmp_x, tmp_y;

// Callback event for detecting mouse-click on images
void CallBackFunc(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "\nLeft button of the mouse is clicked - position (" << x
                  << ", " << y << ")\n" << std::endl;
        tmp_x = x;
        tmp_y = y;
    }
}

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

struct StereoPair {
    Camera cam1;
    Camera cam2;
};


void loadCamFromStream(std::istream &input, Camera &cam)
{
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

bool readStereoCameraFile(const std::string &fileNameP,
                          StereoPair &stereoPair)
{
    int number_of_cameras;
    Camera cam1, cam2;
    std::ifstream ifs(fileNameP.c_str());

    if (ifs) {
        ifs >> number_of_cameras;

        if (number_of_cameras == 2) {
            loadCamFromStream(ifs, cam1);
            loadCamFromStream(ifs, cam2);
            stereoPair.cam1 = cam1;
            stereoPair.cam2 = cam2;
            return true;
        }
    }

    return false;
}

void updateRightLeftImages(sensor_msgs::ImageConstPtr const &msg1, sensor_msgs::ImageConstPtr const &msg2, cv::Mat &imgLeft, cv::Mat &imgRight)
{
    ROS_INFO("Got imgs");
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8);
    imgLeft = cv_ptr1->image;
    imgRight = cv_ptr2->image;
}



//class MyClass {
//public:
//  MyClass() :
//    it_(nh_),
//    orig_image_sub_( it_, "image/orig", 1 ),
//    warp_image_sub_( it_, "image/warp", 1 ),

//    sync( MySyncPolicy( 10 ), orig_image_sub_, warp_image_sub_ )
//  {
//    sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2 ) );
//  }

//  void callback(
//    const sensor_msgs::ImageConstPtr& orig_msg,
//    const sensor_msgs::ImageConstPtr& warp_msg
//  ){
//    // your code here
//  }

//private:
//  ros::NodeHandle nh_;
//  image_transport::ImageTransport it_;

//#if USE_IMAGE_TRANSPORT_SUBSCRIBER_FILTER
//  typedef image_transport::SubscriberFilter ImageSubscriber;
//#else
//  typedef message_filters::Subscriber< sensor_msgs::Image > ImageSubscriber;
//#endif

//  ImageSubscriber orig_image_sub_;
//  ImageSubscriber warp_image_sub_;

//  typedef message_filters::sync_policies::ApproximateTime<
//    sensor_msgs::Image, sensor_msgs::Image
//  > MySyncPolicy;

//  message_filters::Synchronizer< MySyncPolicy > sync;
//};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "balltracker_node");

    cv::Mat leftImg, rightImg;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // TODO topic names
    // Subscribe to image topics using a filter that
    image_transport::SubscriberFilter imgSubLeft(it, "/image_raw", 1);
    image_transport::SubscriberFilter imgSubRight(it, "/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> policy_type;
    //    policy_type policy(10); // Ctor argument is queue size
    message_filters::Synchronizer<policy_type> sync(policy_type(10), imgSubLeft, imgSubRight);
    //    sync.registerCallback(boost::bind(updateRightLeftImages, _1, _2, boost::ref(leftImg), boost::ref(rightImg)));


    //    //Load default calibration and image files
    //    string calibrationFile = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/calibration.txt";
    //    string leftImg = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/left.png";
    //    string rightImg = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/right.png";


    //    //Try to load images and calibration file
    //    Mat img_1, img_2;
    //    img_1 = imread(leftImg, CV_LOAD_IMAGE_COLOR);
    //    img_2 = imread(rightImg, CV_LOAD_IMAGE_COLOR);
    //    if (img_1.empty() || img_2.empty()) {
    //        cout << "Error loading the images" << endl;
    //        return -1;
    //    }

    //    StereoPair stereoCam;
    //    ifstream ifs(calibrationFile.c_str());
    //    if (ifs) {
    //        //Load calibration file
    //        readStereoCameraFile(calibrationFile, stereoCam);
    //    } else {
    //        cout << "Error opening calibration file" << endl;
    //        return -1;
    //    }

    //    //print out the data loaded from the calibration file
    //    stereoCam.cam1.printData();
    //    stereoCam.cam2.printData();

    //    std::cout<<"ProjectionMatrixLeft:\n"<<stereoCam.cam1.projection<<std::endl;
    //    std::cout<<"\nProjectionMatrixRight:\n"<<stereoCam.cam2.projection<<std::endl;


    //    //-------------------
    //    //Compute de 3D point
    //    //-------------------
    //    //Compute the values needed for A and b matrices
    //    Mat Q1Tl = stereoCam.cam1.projection(Range(0,1), Range(0,3));
    //    Mat Q2Tl = stereoCam.cam1.projection(Range(1,2), Range(0,3));
    //    Mat Q3Tl = stereoCam.cam1.projection(Range(2,3), Range(0,3));
    //    Mat q14l = stereoCam.cam1.projection(Range(0,1), Range(3,4));
    //    Mat q24l = stereoCam.cam1.projection(Range(0,1), Range(3,4));
    //    Mat q34l = stereoCam.cam1.projection(Range(0,1), Range(3,4));

    //    Mat Q1Tr = stereoCam.cam1.projection(Range(0,1), Range(0,3));
    //    Mat Q2Tr = stereoCam.cam1.projection(Range(1,2), Range(0,3));
    //    Mat Q3Tr = stereoCam.cam1.projection(Range(2,3), Range(0,3));
    //    Mat q14r = stereoCam.cam1.projection(Range(0,1), Range(3,4));
    //    Mat q24r = stereoCam.cam1.projection(Range(0,1), Range(3,4));
    //    Mat q34r = stereoCam.cam1.projection(Range(0,1), Range(3,4));


    //    //---Test---
    //    Point leftPoint(0,0);
    //    Point rightPoint(0,0);


    //    //Compute A and b matrices
    //    Mat A(4,3,CV_32F);
    //    Mat b(4,1,CV_32F);

    //    A.row(0)=Q1Tl-leftPoint.x*Q3Tl;
    //    A.row(1)=Q2Tl-leftPoint.y*Q3Tl;
    //    A.row(2)=Q1Tr-rightPoint.x*Q3Tr;
    //    A.row(3)=Q2Tr-rightPoint.y*Q3Tr;

    //    b.row(0)=leftPoint.x*q34l-q14l;
    //    b.row(1)=leftPoint.y*q34l-q24l;
    //    b.row(2)=rightPoint.x*q34r-q14r;
    //    b.row(3)=rightPoint.y*q34r-q24r;


    //    //Compute the 3D point
    //    Mat M=(A.t()*A).inv()*A.t()*b;

    //    cout << "M: " << M << endl;

    //    //Take out the translation and rotation part from the Projection matrices
    //    Mat Pxr = stereoCam.cam2.projection(Range(0, 3), Range(0, 3));
    //    Mat pxr = stereoCam.cam2.projection(Range(0, 3), Range(3, 4));

    //    Mat Pxl = stereoCam.cam1.projection(Range(0, 3), Range(0, 3));
    //    Mat pxl = stereoCam.cam1.projection(Range(0, 3), Range(3, 4));

    return EXIT_SUCCESS;
}

//    for( std::size_t i = 0; i < circles.size(); i++ ) {
//         cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//         int radius = cvRound(circles[i][2]);
//         // draw the circle center
//         cv::circle( img, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//         // draw the circle outline
//         cv::circle( img, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//    }

//    cv::namedWindow( "circles", cv::WINDOW_NORMAL );
//    cv::imshow( "circles", img );

//    cv::namedWindow( "gray", cv::WINDOW_NORMAL );
//    cv::imshow( "gray", gray );
//    cv::waitKey();

*/
