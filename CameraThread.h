#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QThread>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <QTime>

#include <QMutex>

///#######################################################
///#######################################################
/// CameraBuffer:

#define CAMBUFFER_DROP_OLDEST    0
#define CAMBUFFER_DROP_NEWEST    1

class CameraBuffer {
public:
    CameraBuffer(int n_frames, int drop_technique = CAMBUFFER_DROP_NEWEST);

    bool     push(cv::Mat &frame, int time_stamp);
    bool     pop(cv::Mat &frame, int &time_stamp);
    double   usage();

private:
    int        max_size;
    int        size;
    int        head;
    int        tail;
    int        drop_technique;
    cv::Mat   *frames;
    int       *times;
};

///#######################################################
///#######################################################
/// CameraThread:

#define BUFFER_SIZE 10

class CameraThread : public QThread
{
    Q_OBJECT
public:
    explicit CameraThread(QObject *parent = 0);

    void     stop();
    void     setup(int device_ID);
    void     setup(QString URL);

    cv::Mat &get_frame(int &time_stamp);
    double   buffer_usage();

signals:

public slots:

private:
    void              run();

    QString           mediaURL;
    int               deviceID;
    cv::VideoCapture  camera;
    CameraBuffer     *buffer;
    QTime             marca_pasos;
    QMutex            camera_mutex;
};

#endif // CAMERATHREAD_H
