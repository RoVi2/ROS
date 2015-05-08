#include "pa10plugin.h"
#include "ui_pa10plugin.h"

#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudio.hpp>

#include "rosnodethread.h"
#include "rosnodethreadimages.h"

Q_EXPORT_PLUGIN2(pa10_plugin, PA10Plugin)

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

PA10Plugin::PA10Plugin()
: RobWorkStudioPlugin("Pa10Plugin", QIcon())
, ui_(new Ui::PA10Plugin)
, ros_thread_(new RosNodeThread)
, ros_thread_images_(new RosNodeThreadImages)
{
	ui_->setupUi(this);
	qRegisterMetaType<rw::math::Q>();
	qRegisterMetaType<rw::math::Transform3D<>>();
	qRegisterMetaType<std::string>();
	qRegisterMetaType<rw::common::Log::LogIndex>();
	qRegisterMetaType<rw::sensor::Image>();
}

PA10Plugin::~PA10Plugin()
{
	if (ros_thread_) {
		ros_thread_->stopNode();

		if (ros_thread_->wait(5000)) {
			delete ros_thread_;
		}
	}

	if (ros_thread_images_) {
		ros_thread_images_->stopNode();

		if (ros_thread_images_->wait(5000)) {
			delete ros_thread_images_;
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
	connect(ui_->btnImages, SIGNAL(pressed()),
			ros_thread_images_, SLOT(startStopImages()));

	connect(ros_thread_, SIGNAL(rwsLogMsg(std::string, rw::common::Log::LogIndex)),
			this, SLOT(rwsLogWrite(std::string, rw::common::Log::LogIndex)));

	connect(ros_thread_, SIGNAL(qUpdated(rw::math::Q)),
			this, SLOT(setPA10Config(rw::math::Q)));

	connect(ros_thread_, SIGNAL(ballPredictedUpdated(rw::math::Transform3D<>)),
			this, SLOT(setBallPredictedTransformation(rw::math::Transform3D<>)));
	connect(ros_thread_, SIGNAL(ballDetectedUpdated(rw::math::Transform3D<>)),
			this, SLOT(setBallDetectedTransformation(rw::math::Transform3D<>)));

	connect(ros_thread_images_, SIGNAL(leftImageUpdated(rw::sensor::Image)),
			this, SLOT(setLeftImage(rw::sensor::Image)));
	connect(ros_thread_images_, SIGNAL(rightImageUpdated(rw::sensor::Image)),
			this, SLOT(setRightImage(rw::sensor::Image)));

	connect(ui_->callbackSpeedSlider, SIGNAL(valueChanged(int)),
			ros_thread_, SLOT(readSlider(int)));
}

void PA10Plugin::open(WorkCell *workcell)
{
	if (!workcell) {
		return;
	}

	workcell_ = workcell;
	state_ = workcell->getDefaultState();
	pa10_= workcell->findDevice<SerialDevice>("PA10");



	if (!pa10_ || pa10_->getDOF() != 7) {
		log().warning() << "PA10 device not found in workcell.\n";
		return;
	}

	ros_thread_->start();
	ros_thread_images_->start();
}

void PA10Plugin::close()
{
	ros_thread_->stopNode();
	ros_thread_images_->stopNode();
}

void PA10Plugin::setPA10Config(Q q)
{
	if (!pa10_) {
		return;
	}

	pa10_->setQ(q, state_);
	getRobWorkStudio()->setState(state_);
}

void PA10Plugin::setBallPredictedTransformation(Transform3D<> transformation){
	//Create a movable frame
	ballPredictedFrame_ = static_cast<MovableFrame*>(workcell_->findFrame("BallPredicted"));
	if (ballDetectedFrame_!=NULL){
		//Move it relative to its parent's parent
		ballPredictedFrame_->moveTo(transformation, ballPredictedFrame_->getParent(), state_);
		//Update the rws
		getRobWorkStudio()->setState(state_);
	}
}

void PA10Plugin::setBallDetectedTransformation(Transform3D<> transformation){
	//Create a movable frame
	ballDetectedFrame_ = static_cast<MovableFrame*>(workcell_->findFrame("BallDetected"));
	if (ballDetectedFrame_!=NULL){
		//Move it relative to its parent's parent
		ballDetectedFrame_->moveTo(transformation, ballDetectedFrame_->getParent(), state_);
		//Update the rws
		getRobWorkStudio()->setState(state_);
	}
}

void PA10Plugin::setLeftImage(rw::sensor::Image image){


}

void PA10Plugin::setRightImage(rw::sensor::Image image){

}

void PA10Plugin::rwsLogWrite(std::string msg, rw::common::Log::LogIndex log_idx)
{
	log().get(log_idx).write(msg);
}

void PA10Plugin::stateChangedListener(State const &state)
{
	state_ = state;
}
