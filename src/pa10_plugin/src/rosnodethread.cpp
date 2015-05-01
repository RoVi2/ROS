#include "rosnodethread.h"

#include <ros/ros.h>

#include "pa10plugin.h"
#include "pa10_plugin/getJointConfig.h"
#include "pa10_plugin/setJointConfig.h"

RosNodeThread::RosNodeThread(QObject *parent)
    : QThread(parent)
{
}

void RosNodeThread::run()
{
    stop_ = false;

    emit rwsLogMsg("Listener node starting...", rw::common::Log::LogIndex::Info);

    int argc = 1;
    char *argv[] = {const_cast<char *>("pa10plugin"), nullptr};
    ros::init(argc, argv, "pa10plugin_node");
    ros::NodeHandle nh;
    //_pointsSubscriber = _nodeHandle.subscribe(TOPIC, 1, &NodeThread::callback, this);

    // ros::ServiceClient scSetJointConfig = nh.serviceClient<pa10_plugin::setJointConfig>("pa10/setJointsConfig");
    ros::ServiceClient scGetJointConfig = nh.serviceClient<pa10_plugin::getJointConfig>("pa10/getJointConfig");
    ros::Rate loop_rate(60);
    pa10_plugin::getJointConfig srv;


    while (nh.ok() && !stop_) {
        if (scGetJointConfig.call(srv)) {
            std::size_t n = srv.response.positions.size();
            rw::math::Q q(n);

            for (unsigned i = 0; i < n; ++i) {
                q[i] = srv.response.positions[i];
            }

            emit qUpdated(q);
        } else {
            // ROS_WARN("Failed call to getJointConfig service.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    emit rwsLogMsg("Listener node stopping...", rw::common::Log::LogIndex::Info);
}

void RosNodeThread::stop()
{
    stop_ = true;
}
