#include "pa10plugin.h"
#include "ui_pa10plugin.h"

#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudio.hpp>

#include "rosnodethread.h"

Q_EXPORT_PLUGIN2(pa10_plugin, PA10Plugin)

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

PA10Plugin::PA10Plugin()
    : RobWorkStudioPlugin("Pa10Plugin", QIcon())
    , ui_(new Ui::PA10Plugin)
    , ros_thread_(new RosNodeThread)
{
    ui_->setupUi(this);
    qRegisterMetaType<rw::math::Q>();
    qRegisterMetaType<std::string>();
    qRegisterMetaType<rw::common::Log::LogIndex>();
}

PA10Plugin::~PA10Plugin()
{
    if (ros_thread_) {
        ros_thread_->stopNode();

        if (ros_thread_->wait(5000)) {
            delete ros_thread_;
        }
    }

    delete ui_;
}

void PA10Plugin::initialize()
{
    // Listen for changes to the workcell kinematics tree.
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&PA10Plugin::stateChangedListener, this, _1), this);

    connect(ui_->btnStartStop, SIGNAL(pressed()),
            ros_thread_, SLOT(startStopRobot()));
    connect(ros_thread_, SIGNAL(rwsLogMsg(std::string, rw::common::Log::LogIndex)),
            this, SLOT(rwsLogWrite(std::string, rw::common::Log::LogIndex)));
    connect(ros_thread_, SIGNAL(qUpdated(rw::math::Q)),
            this, SLOT(setPA10Config(rw::math::Q)));
    connect(ui_->callbackSpeedSlider, SIGNAL(valueChanged(int)),
            ros_thread_, SLOT(readSlider(int)));
}

void PA10Plugin::open(WorkCell *workcell)
{
    if (!workcell) {
        return;
    }


    state_ = workcell->getDefaultState();
    pa10_= workcell->findDevice<SerialDevice>("PA10");

    if (!pa10_ || pa10_->getDOF() != 7) {
        log().warning() << "PA10 device not found in workcell.\n";
        return;
    }

    ros_thread_->start();
}

void PA10Plugin::close()
{
    ros_thread_->stopNode();
}

void PA10Plugin::setPA10Config(Q q)
{
    if (!pa10_) {
        return;
    }

    pa10_->setQ(q, state_);
    getRobWorkStudio()->setState(state_);
}

void PA10Plugin::rwsLogWrite(std::string msg, rw::common::Log::LogIndex log_idx)
{
    log().get(log_idx).write(msg);
}

void PA10Plugin::stateChangedListener(State const &state)
{
    state_ = state;
}
