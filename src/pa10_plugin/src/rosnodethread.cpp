#include "rosnodethread.h"

#include <ros/ros.h>

#include "pa10plugin.h"
#include "pa10_dummy/getJointConfig.h"

#define PARAM_FRAME_RATE "/frame_rate"

RosNodeThread::RosNodeThread(QObject *parent)
    : QThread(parent)
{
}

void RosNodeThread::run()
{
    stop_ = false;
    emit rwsLogMsg("ROS node starting...\n", rw::common::Log::LogIndex::Info);
    int argc = 1;
    char *argv[] = {const_cast<char *>("pa10_plugin"), nullptr};
    ros::init(argc, argv, "pa10_plugin_node");
    ros::NodeHandle nh;
    ros::ServiceClient sc_get_joint_conf = nh.serviceClient<pa10_dummy::getJointConfig>("pa10/getJointConfig");
    
    int frame_rate = 1;
    pa10_dummy::getJointConfig srv;

    while (nh.ok() && !stop_) {
        //Get the globar frame rate
        ros::param::get(PARAM_FRAME_RATE, frame_rate);
        ros::Rate loop_rate(frame_rate);

        if (sc_get_joint_conf.call(srv)) {
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

    emit rwsLogMsg("ROS node stopping...\n", rw::common::Log::LogIndex::Info);
}

void RosNodeThread::stop()
{
    stop_ = true;
}
