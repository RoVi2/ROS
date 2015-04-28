#include "rwsplugin.hpp"
#include "ui_rwsplugin.h"

#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudio.hpp>

#include "nodethread.hpp"

using namespace rw::models;
using namespace rw::kinematics;

Q_EXPORT_PLUGIN2(rws, RwsPlugin)

RwsPlugin::RwsPlugin()
    : RobWorkStudioPlugin("RwsPlugin", QIcon())
    , ui_(new Ui::RwsPlugin)
    //, node_(new NodeThread)
{
    ui_->setupUi(this);
    connect(ui_->btnLoadBg, SIGNAL(pressed()), this, SLOT(test()));
}

RwsPlugin::~RwsPlugin()
{
    delete ui_;
}

void RwsPlugin::initialize()
{
    // Listen for changes to the workcell kinematics tree
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&RwsPlugin::stateChangedListener, this, _1), this);
}

void RwsPlugin::open(WorkCell *workcell)
{
    if (!workcell) {
        return;
    }
    //node_->start();

    NodeThread * node = new NodeThread(workcell);
    node->run();
}

void RwsPlugin::close()
{
}

void RwsPlugin::stateChangedListener(State const &state)
{
    state_ = state;
}

void RwsPlugin::test()
{
    log().info() << "Test button pressed!\n";
}
