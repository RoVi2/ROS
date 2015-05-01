#include "pa10plugin.h"
#include "ui_pa10plugin.h"

#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudio.hpp>

#include "rosnodethread.h"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

Q_EXPORT_PLUGIN2(pa10_plugin, PA10Plugin)

PA10Plugin::PA10Plugin()
    : RobWorkStudioPlugin("Pa10Plugin", QIcon())
    , ui_(new Ui::PA10Plugin)
    , rosThread_(new RosNodeThread)
{
    ui_->setupUi(this);
    qRegisterMetaType<Q>();
    qRegisterMetaType<std::string>();
    qRegisterMetaType<Log::LogIndex>();
}

PA10Plugin::~PA10Plugin()
{
    if (rosThread_) {
        rosThread_->stop();
        rosThread_->wait();
        delete rosThread_;
    }

    delete ui_;
}

void PA10Plugin::initialize()
{
    // Listen for changes to the workcell kinematics tree.
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&PA10Plugin::stateChangedListener, this, _1), this);

    // Connect UI.
    connect(ui_->btnTest, SIGNAL(pressed()), this, SLOT(test()));

    // Connect ROS node thread.
    connect(rosThread_, SIGNAL(rwsLogMsg(std::string, rw::common::Log::LogIndex)),
            this, SLOT(rwsLogWrite(std::string, rw::common::Log::LogIndex)));
    connect(rosThread_, SIGNAL(qUpdated(rw::math::Q)),
            this, SLOT(setPA10Config(rw::math::Q)));
}

void PA10Plugin::open(WorkCell *workcell)
{
    if (!workcell) {
        return;
    }

    pa10_ = workcell->findDevice("PA10").scast<SerialDevice>();

    if (!pa10_) {
        log().warning() << "PA10 device not found in workcell.";
        return;
    }

    rosThread_->start();
}

void PA10Plugin::close()
{
    rosThread_->stop();
}

void PA10Plugin::setPA10Config(Q q)
{
    if (!pa10_) {
        return;
    }

    State state;
    pa10_->setQ(q, state);
    getRobWorkStudio()->setState(state);
}

void PA10Plugin::rwsLogWrite(std::string msg, rw::common::Log::LogIndex logIdx)
{
    log().get(logIdx).write(msg);
}

void PA10Plugin::stateChangedListener(State const &state)
{
    state_ = state;
}

void PA10Plugin::test()
{
    log().info() << "Test button pressed!\n";
}
