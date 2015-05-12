#include "syncedimages.h"

SyncedImages::SyncedImages(image_transport::ImageTransport it,
                           std::string const &left_topic,
                           std::string const &right_topic)
	:
      sub_left_(it, left_topic, 4, image_transport::TransportHints("compressed")) // Queue size of 4 is shown to be reasonable.
    , sub_right_(it, right_topic, 4, image_transport::TransportHints("compressed")) // Smaller queue reduces performance.
    , sync_(ApproxPolicyType(1), sub_left_, sub_right_)
    , updated_(false)
    , time_last_update_(0)
{

    sync_.registerCallback(boost::bind(&SyncedImages::imageCb, this, _1, _2));
}

bool SyncedImages::updated()
{
    boost::lock_guard<boost::mutex> guard(mutex_);
    return updated_;
}

void SyncedImages::images(cv::Mat &left, cv::Mat &right)
{
    boost::lock_guard<boost::mutex> guard(mutex_);
    time_last_update_ = timer_.elapsed().user;
    img_left_.copyTo(left);
    img_right_.copyTo(right);
    updated_ = false;
}

boost::timer::nanosecond_type SyncedImages::timeSinceLastUpdate()
{
    boost::lock_guard<boost::mutex> guard(mutex_);
    return timer_.elapsed().user - time_last_update_;
}

void SyncedImages::imageCb(sensor_msgs::ImageConstPtr const &left, sensor_msgs::ImageConstPtr const &right)
{
    boost::lock_guard<boost::mutex> guard(mutex_);
    img_left_ = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::RGB8)->image;
    img_right_ = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::RGB8)->image;
    updated_ = true;
}
