#ifndef RWSPLUGIN_HPP
#define RWSPLUGIN_HPP

#include <rw/kinematics/State.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

class NodeThread;

namespace Ui
{
class RwsPlugin;
}

class RwsPlugin : public rws::RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES(rws::RobWorkStudioPlugin)

public:
    RwsPlugin();
    ~RwsPlugin() override;
    void initialize() override;
    void open(rw::models::WorkCell *workcell) override;
    void close() override;

private slots:
    void stateChangedListener(rw::kinematics::State const &state);
    void test();

private:
    Ui::RwsPlugin *ui_;
    //NodeThread *node_;
    rw::kinematics::State state_;
};

#endif // RWSPLUGIN_HPP
