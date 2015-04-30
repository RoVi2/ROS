#include "pa10plugin.h"
#include "ui_pa10plugin.h"

#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudio.hpp>

#include "nodethread.h"

using namespace rw::models;
using namespace rw::kinematics;

Q_EXPORT_PLUGIN2(pa10_plugin, Pa10Plugin)

Pa10Plugin::Pa10Plugin()
    : RobWorkStudioPlugin("Pa10Plugin", QIcon())
    , ui_(new Ui::Pa10Plugin)
    , nodeThread_(new NodeThread)
{
    ui_->setupUi(this);
}

Pa10Plugin::~Pa10Plugin()
{
    if (nodeThread_) {
        nodeThread_->stop();
        nodeThread_->wait();
        delete nodeThread_;
    }

    delete ui_;
}

void Pa10Plugin::initialize()
{
    // Listen for changes to the workcell kinematics tree.
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&Pa10Plugin::stateChangedListener, this, _1), this);

    // Connect UI
    QObject::connect(ui_->btnTest, SIGNAL(pressed()), this, SLOT(test()));
    QObject::connect(nodeThread_, SIGNAL(logMsg()), this, SLOT(rwsLogWrite()));

}

void Pa10Plugin::open(WorkCell *workcell)
{
    if (!workcell) {
        return;
    }

    nodeThread_->start();
}

void Pa10Plugin::close()
{
    nodeThread_->stop();
}

void Pa10Plugin::rwsLogWrite(std::string msg, rw::common::Log::LogIndex logIdx)
{
    log().get(logIdx).write(msg);
}

void Pa10Plugin::stateChangedListener(State const &state)
{
    state_ = state;
}

void Pa10Plugin::test()
{
    log().info() << "Test button pressed!\n";
}
