#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#define PUBLISHER  "monocamera_camera/image"

int main(int argc, char** argv)
 {

   ros::init(argc, argv, "monocamera_publisher");
   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise(PUBLISHER, 1);

   cv::Mat imageA = cv::imread(ros::package::getPath("monocamera_dummy")+"/ownA.ppm");
   cv::Mat imageB = cv::imread(ros::package::getPath("monocamera_dummy")+"/ownB.ppm");
   cv::Mat imageC = cv::imread(ros::package::getPath("monocamera_dummy")+"/ownC.ppm");
   cv::Mat imageD = cv::imread(ros::package::getPath("monocamera_dummy")+"/ownD.ppm");

   if(imageA.empty() || imageB.empty() || imageC.empty() || imageD.empty()){
      ROS_ERROR("no image loaded"); 
      return 0;   
   }

   // convert OpenCV image to ROS message
   ros::Rate loop_rate(5);
   int count = 0;
   while (nh.ok()) {
     cv_bridge::CvImage cvi;
     cvi.header.stamp = ros::Time::now();
     cvi.header.frame_id = "image";
     cvi.encoding = "bgr8";
     if(count % 4 == 0)
        cvi.image = imageA;
     else if(count % 4 == 1)
        cvi.image = imageB;
     else if(count % 4 == 2)
        cvi.image = imageC;
     else if(count % 4 == 3)
        cvi.image = imageD;

     sensor_msgs::Image im;
     cvi.toImageMsg(im);

     pub.publish(im);
     ros::spinOnce();
     loop_rate.sleep();
     count++;
   }
 }

