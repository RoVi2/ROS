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
    stopNode_ = false;
    emit rwsLogMsg("ROS node starting...\n", rw::common::Log::LogIndex::Info);
    int argc = 1;
    char *argv[] = {const_cast<char *>("pa10_plugin"), nullptr};
    ros::init(argc, argv, "pa10_plugin_node");
    ros::NodeHandle nh;
    ros::ServiceClient sc_get_joint_conf = nh.serviceClient<pa10_dummy::getJointConfig>("pa10/getJointConfig");
    int frame_rate = 1;
    pa10_dummy::getJointConfig srv;

    while (nh.ok() && !stopNode_) {
        if (!stopRobot_){
            // Get the global frame rate
            ros::param::get(PARAM_FRAME_RATE, frame_rate);
            ros::Rate loop_rate(frame_rate*10);
            //ros::Rate loop_rate(frame_rate);

            if (sc_get_joint_conf.call(srv)) {
                std::size_t n = srv.response.positions.size();
                rw::math::Q q(n);

                emit rwsLogMsg("Q: ", rw::common::Log::LogIndex::Info);
                for (unsigned char i = 0; i < n; ++i){
                    q(i) = srv.response.positions[i]/rw::math::Rad2Deg;
                    std::stringstream q_string;
                    q_string << q(i) << ", ";
                    emit rwsLogMsg(q_string.str().c_str(), rw::common::Log::LogIndex::Info);
                }
                emit rwsLogMsg("\n", rw::common::Log::LogIndex::Info);


                emit qUpdated(q);
            } else {
                emit rwsLogMsg("Failed call to getJointConfig service.\n", rw::common::Log::LogIndex::Info);
                ROS_WARN("Failed call to getJointConfig service.");
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    emit rwsLogMsg("ROS node stopping...\n", rw::common::Log::LogIndex::Info);
}

void RosNodeThread::stopNode()
{
    emit rwsLogMsg("Node Stoped!\n", rw::common::Log::LogIndex::Info);
    stopNode_ = true;
}

void RosNodeThread::startStopRobot(){
    if (!stopRobot_){
        emit rwsLogMsg("Robot Stoped!\n", rw::common::Log::LogIndex::Info);
        stopRobot_ = true;
    }
    else{
        emit rwsLogMsg("Robot Started!\n", rw::common::Log::LogIndex::Info);
        stopRobot_ = false;
    }
}

void RosNodeThread::readSlider(int loopRate){
    //Due to the slider only lets integer numbers and we are interested in decimals
    loopRate_ = loopRate;
}
