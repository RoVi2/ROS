#ifndef PA10PLUGIN_H
#define PA10PLUGIN_H

#include <rw/math.hpp>
#include <rw/kinematics.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

class RosNodeThread;

namespace Ui
{
class PA10Plugin;
}

class PA10Plugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES(rws::RobWorkStudioPlugin)

public:
    PA10Plugin();
    ~PA10Plugin() override;
    void initialize() override;
    void open(rw::models::WorkCell *workcell) override;
    void close() override;

public slots:
    void setPA10Config(rw::math::Q q);
    void setBallPredictedTransformation(rw::math::Transform3D<>);
    void setBallDetectedTransformation(rw::math::Transform3D<>);
    void rwsLogWrite(std::string msg, rw::common::Log::LogIndex log_idx);

private slots:
    void stateChangedListener(rw::kinematics::State const &state);

private:
    Ui::PA10Plugin *ui_;
    RosNodeThread *ros_thread_;
    rw::kinematics::State state_;
    rw::models::SerialDevice::Ptr pa10_;
    rw::models::WorkCell::Ptr workcell_;
    rw::kinematics::MovableFrame* ballPredictedFrame_;
    rw::kinematics::MovableFrame* ballDetectedFrame_;
};

Q_DECLARE_METATYPE(rw::math::Q)
Q_DECLARE_METATYPE(rw::math::Transform3D<>)
Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(rw::common::Log::LogIndex)

#endif // PA10PLUGIN_H
