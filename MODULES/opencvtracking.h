#ifndef OPENCVTRACKING_H
#define OPENCVTRACKING_H

#include "ModuleInterface.h"
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/blobtrack.hpp>

using namespace cv;

class CvBlobTrackerAutoc;


class OpencvTracking: public ModuleInterface
{
public:
    OpencvTracking(Datapool *i_data);
    ~OpencvTracking();

    bool setParameters(QDomNode& config);
    bool init();
    bool run();
    bool updateParameters();

private:
    bool first;
    QString bgImName;
    Mat qImage2Mat(const QImage& qimage);
    QImage Mat2QImage(const Mat& mat);
    CvBlobTrackerAutoc*          pTracker;
    bool debug;
    void OpencvDebug(IplImage & pImg);
};

#endif // OPENCVTRACKING_H
