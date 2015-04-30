#ifndef NODETHREAD_H
#define NODETHREAD_H

#include <QThread>

#include <rw/common/Log.hpp>
#include <rw/math/Q.hpp>

class NodeThread : public QThread
{
    Q_OBJECT

public:
    NodeThread(QObject *parent = nullptr);
    void run() override;
    void stop();

signals:
    void qUpdated(rw::math::Q q);
    void logMsg(std::string msg, rw::common::Log::LogIndex logIdx);

private:
    bool stop_;
};

#endif // NODETHREAD_H
