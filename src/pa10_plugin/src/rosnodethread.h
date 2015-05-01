#ifndef ROSNODETHREAD_H
#define ROSNODETHREAD_H

#include <QThread>

#include <rw/common/Log.hpp>
#include <rw/math/Q.hpp>

class RosNodeThread : public QThread
{
    Q_OBJECT

public:
    RosNodeThread(QObject *parent = nullptr);
    void run() override;
    void stop();

signals:
    void qUpdated(rw::math::Q q);
    void rwsLogMsg(std::string msg, rw::common::Log::LogIndex logIdx);

private:
    bool stop_;
};

#endif // ROSNODETHREAD_H
