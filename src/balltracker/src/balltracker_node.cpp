#include <cmath>
#include <vector>
#include <tuple>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "syncedimages.h"

bool findWhiteBall(cv::Mat const &img,
                   cv::Point2d &center,
                   double &circumference,
                   double &area)
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
    cv::inRange(hsv_img, cv::Scalar(0, 0, 245), cv::Scalar(30, 30, 255), filt_bin_img);

    // Find contours.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(filt_bin_img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    // Examine each contour. Only large ones are recognized as the ball.
    // TODO Only the center of first the first contour fitting is returned.
    for (auto const &contour : contours) {
        double circumference_tmp = cv::arcLength(contour, true);
        double area_tmp = cv::contourArea(contour);

        if (circumference_tmp > 370 && area_tmp > 4700) {
            cv::Moments mom = cv::moments(contour, true);
            area = area_tmp;
            circumference = circumference_tmp;
            center = cv::Point2d(mom.m10 / mom.m00, mom.m01 / mom.m00); // Mass center
            return true;
        }
    }

    return false;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "balltracker_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_left = it.advertise("balltracker/image/left", 1);
    image_transport::Publisher pub_right = it.advertise("balltracker/image/right", 1);
    SyncedImages si(it, "stereo_camera/left/image_raw", "stereo_camera/right/image_raw");
    ros::Rate loop_rate(10); // 10 hz
    cv::Mat img_left, img_right;
    bool success_l, success_r;
    cv::Point2d center_l, center_r;
    double circum_l, circum_r, area_l, area_r;

    while (nh.ok()) {
        if (si.updated()) {
            si.images(img_left, img_right);
            success_l = findWhiteBall(img_left, center_l, circum_l, area_l);
            success_r = findWhiteBall(img_right, center_r, circum_r, area_r);

            if (success_l) {
                cv::circle(img_left, center_l, circum_l / (2 * M_PI), cv::Scalar(255, 0, 0), 3);
            }

            if (success_r) {
                cv::circle(img_right, center_r, circum_r / (2 * M_PI), cv::Scalar(255, 0, 0), 3);
            }

            sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img_left).toImageMsg();
            sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img_right).toImageMsg();
            pub_left.publish(msg_left);
            pub_right.publish(msg_right);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
