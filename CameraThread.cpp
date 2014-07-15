#include "CameraThread.h"

#include "VideoAnalysis.h"

///#######################################################
///#######################################################
/// CameraBuffer:

CameraBuffer::CameraBuffer(int n_frames, int drop_technique) {
    max_size             = n_frames;
    size                 = 0;
    head                 = 0;
    tail                 = 0;
    this->drop_technique = drop_technique;
    frames               = new cv::Mat[max_size];
    times                = new int[max_size];
}

bool CameraBuffer::push(cv::Mat &frame, int time_stamp) {
    if(drop_technique == CAMBUFFER_DROP_NEWEST) {
        if(size == max_size)
            return false; // is full
    } else if(drop_technique == CAMBUFFER_DROP_OLDEST) {
        if(size == max_size) { // still full, but now oldest frame get lost
            frame.copyTo(frames[head]);
            times[head] = time_stamp;
            tail = (tail+1)%max_size;
            head = (head+1)%max_size;
            return true;
        }
    } else
        return false; // drop_technique not correct

    // common task for "if" with no direct return:
    frame.copyTo(frames[head]);
    times[head] = time_stamp;
    head = (head+1)%max_size;
    size++;
    return true;
}

bool CameraBuffer::pop(cv::Mat &frame, int &time_stamp) {
    if(head==tail && (size==0))
        return false; // is empty

    frames[tail].copyTo(frame);
    time_stamp = times[tail];
    tail = (tail+1)%max_size;
    size--;
    return true;
}

double CameraBuffer::usage() {
    return (double(size)/double(max_size));
}

///#######################################################
///#######################################################
/// CameraThread:

CameraThread::CameraThread(QObject *parent) : QThread(parent) {
    mediaURL     = "";
    deviceID     = 0;
    buffer       = new CameraBuffer(BUFFER_SIZE, CAMBUFFER_DROP_OLDEST);
    marca_pasos.start(); // time messure for frames.
}

void CameraThread::run() {
    cv::Mat frame;

    while(camera.isOpened()) {
        camera_mutex.lock();
        camera >> frame;
        camera_mutex.unlock();
        buffer->push(frame, marca_pasos.elapsed());
    }
}

void CameraThread::stop() {
    //running_flag = false;
    std::cout << "Disconnecting Camera...";
    camera_mutex.lock();
    camera.release();
    if(!camera.isOpened())
        std::cout << "OK" << std::endl;
    else
        std::cout << "¡¡FAILED!!" << std::endl;
    camera_mutex.unlock();
}

void CameraThread::setup(int device_ID) {
    deviceID = device_ID;
    camera.open(deviceID);
    if(!camera.isOpened())
        AppendToLog("Error: trying to connect to local device # "+QString::number(deviceID));
}

void CameraThread::setup(QString URL) {
    mediaURL = URL;
    camera.open(mediaURL.toStdString());
    if(!camera.isOpened())
        AppendToLog("Error: trying to connect to remote device # "+URL);
}

cv::Mat &CameraThread::get_frame(int &time_stamp) {
    static cv::Mat current;
    buffer->pop(current, time_stamp);
    return current;
}

double CameraThread::buffer_usage() {
    return buffer->usage();
}
