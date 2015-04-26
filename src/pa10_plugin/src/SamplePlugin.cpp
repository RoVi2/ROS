#include "SamplePlugin.hpp"

using namespace rwlibs::task;


namespace {
	bool initializeRos() {
#ifndef NO_ROS
		char** argv = NULL;
		int argc = 0;
		ros::init(argc, argv, "PA10RWSPlugin");
#endif
		return true;
	}
}

void AddIPL98ErrorHistory(char const *) {
}


SamplePlugin::SamplePlugin(): 
RobWorkStudioPlugin("RobotControl", QIcon("")),
_rosInitialized(initializeRos())
{
	
    QWidget *widg = new QWidget(this);
	QVBoxLayout *lay = new QVBoxLayout(widg);
	widg->setLayout(lay);
	this->setWidget(widg);

    // Setup ToolBar
    _comWidget = new SampleWidget(this);
    lay->addWidget(_comWidget);
}

SamplePlugin::~SamplePlugin()
{

}

void SamplePlugin::initialize() {
    getRobWorkStudio()->genericEvent().add(boost::bind(&SamplePlugin::genericEvent, this, _1), this);    
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);    
}

void SamplePlugin::stateChangedListener(const rw::kinematics::State &state)
{
	_comWidget->stateChangedListener(state); 
}
void SamplePlugin::genericEvent(const std::string& str) {
 
}


void SamplePlugin::open(rw::models::WorkCell* workcell) {
#ifndef NO_ROS 
    _comWidget->initialize(workcell, getRobWorkStudio(), &_nodeHandle);
#else    
    _comWidget->initialize(workcell, getRobWorkStudio(), NULL);
#endif
}   

void SamplePlugin::close() {

}

Q_EXPORT_PLUGIN(SamplePlugin); 
