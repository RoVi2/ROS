#include <boost/thread.hpp>
#include <boost/timer/timer.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

/**
 * \brief Class subscribes to two ROS image topics and keeps updates syncronized.
 */
class SyncedImages
{
public:
    /**
     * \brief Constructor.
     *
     * \param it The image transport to use for subscriptions.
     * \param left_cam_topic The name of the left camera topic.
     * \param right_cam_topic The name of the right camera topic.
     */
    SyncedImages(image_transport::ImageTransport it,
                 std::string const &left_topic,
                 std::string const &right_topic);

    /**
     * \brief Check if both images have been updated since last calling images().
     *
     * \return True if images have been updated, false otherwise.
     */
    bool updated();

    /**
     * \brief Get the latest syncronized images.
     *
     * \param left A Matrix to copy the left image into.
     * \param right A Matrix to copy the right image into.
     */
    void images(cv::Mat &left, cv::Mat &right);

    /**
     * \brief Get the time since images were last updated synchronously.
     *
     * \return Time since last update in nano seconds.
     */
    boost::timer::nanosecond_type timeSinceLastUpdate();

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxPolicyType;

    image_transport::SubscriberFilter sub_left_;
    image_transport::SubscriberFilter sub_right_;
    message_filters::Synchronizer<ApproxPolicyType> sync_;
    bool updated_;
    boost::timer::nanosecond_type time_last_update_;
    boost::timer::cpu_timer timer_;
    boost::mutex mutex_;
    cv::Mat img_left_;
    cv::Mat img_right_;

    void imageCb(sensor_msgs::ImageConstPtr const &left, sensor_msgs::ImageConstPtr const &right);
};
