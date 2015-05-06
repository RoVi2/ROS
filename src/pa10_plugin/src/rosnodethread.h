#ifndef ROSNODETHREAD_H
#define ROSNODETHREAD_H

#include <atomic>

#include <QThread>

#include <rw/common/Log.hpp>
#include <rw/math.hpp>

class RosNodeThread : public QThread
{
    Q_OBJECT

public:
    RosNodeThread(QObject *parent = nullptr);
    void run() override;
    void stopNode();

public slots:
    void startStopRobot();
    void readSlider(int loopRate);

signals:
    void qUpdated(rw::math::Q q);
    void ballPredictedUpdated(rw::math::Transform3D<> transformation);
    void ballDetectedUpdated(rw::math::Transform3D<> transformation);
    void rwsLogMsg(std::string msg, rw::common::Log::LogIndex log_idx);

private:
    std::atomic<bool> stopNode_;
    bool stopRobot_;
    int loopRate_;
};

#endif // ROSNODETHREAD_H
