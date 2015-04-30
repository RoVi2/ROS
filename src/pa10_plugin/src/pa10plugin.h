#ifndef PA10PLUGIN_H
#define PA10PLUGIN_H

#include <rw/kinematics/State.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

class NodeThread;

namespace Ui
{
class Pa10Plugin;
}

class Pa10Plugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES(rws::RobWorkStudioPlugin)

public:
    Pa10Plugin();
    ~Pa10Plugin() override;
    void initialize() override;
    void open(rw::models::WorkCell *workcell) override;
    void close() override;

public slots:
    void rwsLogWrite(std::string msg, rw::common::Log::LogIndex logIdx);

private slots:
    void stateChangedListener(rw::kinematics::State const &state);
    void test();

private:
    Ui::Pa10Plugin *ui_;
    NodeThread *nodeThread_;
    rw::kinematics::State state_;
};

#endif // PA10PLUGIN_H
