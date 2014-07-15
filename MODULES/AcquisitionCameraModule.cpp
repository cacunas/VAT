#include "AcquisitionCameraModule.h"

#include "VideoAnalysis.h"

AcquisitionCameraModule::AcquisitionCameraModule(Datapool *i_data) : ModuleInterface(i_data),
    mediaType("LocalCamera"),
    mediaURL(""),
    device_id(0),
    cameraThread(NULL),
    server_ip(""),
    server_port(""),
    protocol(""),
    metafile(""),
    user(""),
    password("")
{
}

AcquisitionCameraModule::~AcquisitionCameraModule() {
    if(cameraThread != NULL) {
        cameraThread->stop();
    }
}

bool AcquisitionCameraModule::setParameters(QDomNode &config) {
    QDomNode n, m;
    // if "AcquisitionCameraModule" label isn't setted up...
    if(config.isNull()) {
        // we assume mediaType = LocalCamera
        mediaType = "LocalCamera";
    } else {
        if( ( n = XmlCommon::getParameterNode("mediaType", config)).isNull() )
            // we assume mediaType = LocalCamera
            mediaType = "LocalCamera";
        else {
            if( (mediaType = QString::fromStdString(XmlCommon::getParameterValue(n).toStdString())) == "LocalCamera") {
                if( (m = XmlCommon::getParameterNode("deviceID", n)).isNull()) {
                    device_id = 0;
                } else {
                    device_id = XmlCommon::getParameterValue(m).toInt();
                }
            } else if(mediaType == "RemoteCamera" ) {
                if( !(m = XmlCommon::getParameterNode("url", n)).isNull()) {
                    mediaURL = XmlCommon::getParameterValue(m);
                } else { // if null, we'll compose it..
                    //--
                    if( (m = XmlCommon::getParameterNode("server_IP", n)).isNull()) {
                        server_ip = "200.1.17.81";
                    } else {
                        server_ip = XmlCommon::getParameterValue(m);
                    }
                    //--
                    if( (m = XmlCommon::getParameterNode("server_Port", n)).isNull()) {
                        server_port = "554";
                    } else {
                        server_port = XmlCommon::getParameterValue(m);
                    }
                    //--
                    if( (m = XmlCommon::getParameterNode("protocol", n)).isNull()) {
                        protocol = "rtsp";
                    } else {
                        protocol = XmlCommon::getParameterValue(m);
                    }
                    //--
                    if( (m = XmlCommon::getParameterNode("metafile", n)).isNull()) {
                        metafile = "/mpeg4/media.amp";
                    } else {
                        metafile = XmlCommon::getParameterValue(m);
                    }
                    //--
                    if( (m = XmlCommon::getParameterNode("user", n)).isNull()) {
                        user = "VAT_Visor";
                    } else {
                        user = XmlCommon::getParameterValue(m);
                    }
                    //--
                    if( (m = XmlCommon::getParameterNode("password", n)).isNull()) {
                        password = "visor";
                    } else {
                        password = XmlCommon::getParameterValue(m);
                    }
                    //---------------------
                    if(protocol == "rtsp") {
                        mediaURL = "rtsp://"+user+":"+password+"@"+server_ip+":"+server_port+metafile;
                    }
                }
            } else
                AppendToLog("Unrecognized media type");

        }

    }

    addParameter("mediaType", mediaType, "QString");
    addParameter("deviceID", QString::number(device_id), "int");
    addParameter("mediaURL", mediaURL, "QString");
    return true;
}

bool AcquisitionCameraModule::init()
{
    if(cameraThread == NULL) {
        cameraThread = new CameraThread();

        if(mediaType == "LocalCamera") {
            cameraThread->setup(device_id);
        } else if(mediaType == "RemoteCamera") {
            cameraThread->setup(mediaURL);
        } else {
            AppendToLog("Unrecognized media type");
        }
        cameraThread->start(); // with inheritated priority
    } else {
        AppendToLog("Reinitializing AcquisitionCameraModule?");
    }

    return true;
}

bool AcquisitionCameraModule::run() {
    int time_frame;
    static int f_number = 0;

    QImage *current = Mat2QImage(cameraThread->get_frame(time_frame));


    // setted as m_saveCurrent case:
    m_data->frameNumber = f_number++;

    if(m_data->currentImage != NULL) {
        if(m_data->previousImage != NULL)
            delete m_data->previousImage;
        m_data->previousImage = m_data->currentImage;
        m_data->currentImage  = current;
    } else {
        m_data->currentImage  = current;
        m_data->previousImage = NULL;
    }

    // store time data of frame:
    //m_data->currentHeader->ts = mseconds2TimeStamp(time_frame);

    AppendToLog("CameraBuffer usage: "+QString::number(cameraThread->buffer_usage()*100)+"%");

    return true;
}

bool AcquisitionCameraModule::updateParameters() {
    // wait for it...
    return true;
}

QImage *AcquisitionCameraModule::Mat2QImage(cv::Mat &mat) {
    QImage *temp = NULL;

    if(mat.type() == CV_8UC3) {
        temp = new QImage(mat.cols, mat.rows, QImage::Format_RGB32);

        uchar *a = temp->bits();
        uchar *b = mat.data;

        // I had to use this (2 for) to transform 3 channels to 4, since
        // the program is using RGB32 format for QImage:
        for(int j=0; j<mat.rows; j++)
            for(int i=0; i<mat.cols; i++) {
                a[j*temp->bytesPerLine() + i*4 + 0] =
                        b[j*mat.step + i*3 + 0];
                a[j*temp->bytesPerLine() + i*4 + 1] =
                        b[j*mat.step + i*3 + 1];
                a[j*temp->bytesPerLine() + i*4 + 2] =
                        b[j*mat.step + i*3 + 2];
                a[j*temp->bytesPerLine() + i*4 + 3] = 255;
            }
    } else {
        AppendToLog("Mat format not supported to QImage");
    }
    return temp;
}

TimeStamp AcquisitionCameraModule::mseconds2TimeStamp(int msec) {
    TimeStamp p;

    p.millisecond = 0;
    p += msec;

    return p;
}
