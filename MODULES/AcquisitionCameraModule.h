#ifndef ACQUISITIONCAMERAMODULE_H
#define ACQUISITIONCAMERAMODULE_H

#include "Datapool.h"
#include "ModuleInterface.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "CameraThread.h"

class AcquisitionCameraModule : public ModuleInterface {
public:
    AcquisitionCameraModule(Datapool *i_data);
    ~AcquisitionCameraModule();

    //Set module configuration parameters:
    bool setParameters(QDomNode &config);

    //
    bool init();

    //
    bool run();

    //
    bool updateParameters();


    QImage            *Mat2QImage(cv::Mat &img);
    TimeStamp          mseconds2TimeStamp(int msec);

private:
    QString           mediaType;
    QString           mediaURL;
    int               device_id;
    CameraThread     *cameraThread;

    QString           server_ip;
    QString           server_port;
    QString           protocol;
    QString           metafile;
    QString           user;
    QString           password;


};

#endif // ACQUISITIONCAMERAMODULE_H
