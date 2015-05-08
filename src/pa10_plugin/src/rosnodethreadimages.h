#ifndef ROSNODETHREADIMAGES_H
#define ROSNODETHREADIMAGES_H

#include <atomic>

#include <QThread>

#include <rw/sensor.hpp>

class RosNodeThreadImages : public QThread
{
    Q_OBJECT

public:
    RosNodeThreadImages(QObject *parent = nullptr);
    void run() override;
    void stopNode();

public slots:
    void startStopImages();

signals:
    void leftImageUpdated(rw::sensor::Image image);
    void rightImageUpdated(rw::sensor::Image image);
    void rwsLogMsg(std::string msg, rw::common::Log::LogIndex log_idx);

private:
    std::atomic<bool> stopNode_;
    bool stopImages_;
    int loopRate_;
};

#endif // ROSNODETHREADIMAGES_H
