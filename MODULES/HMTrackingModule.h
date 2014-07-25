#ifndef HMTRACKINGMODULE_H
#define HMTRACKINGMODULE_H

#include <QImage>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include <QColor>
#include <QMessageBox>

#include "image_display.h"

#include "Datapool.h"
#include "ModuleInterface.h"

typedef std::vector<int> hist;

using namespace std;

class HMTrackingModule : public ModuleInterface
{
public:
    HMTrackingModule(Datapool* i_data);
    ~HMTrackingModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

private:
    bool firstTime;
    float alpha, beta;
    float A_p[4];// histogram intensity peaks (R,G,B,Gray)
    float A_t[4];// intensity thresholds (R,G,B,Gray)

    //Convert QImage to cv::Mat
    cv::Mat qimage_to_mat_cpy(QImage* img, int format);

    //Calculates Intensity Histograms per channel
    vector<hist> calculateHistograms(QImage* img);

    //Calculates first and second statistical moments of the intesity histograms
    vector<float> calculateMoments(vector<hist> channels);

    //Calculate channels peaks using paper proposed criterias
    void calculatePeaks(vector<hist> channels);

    //Calculate thresholds
    void calculateThresholds(vector<hist> ch);

    //Store grass area of the image on bgImage using a binary classifier
    void GrassClassifier();

    //Get the line on the grass selection usign Hough Transform
    void Line_detect();

    //Aplly a filter
    void ApplyFilter(QImage *f_in, QImage *f_out);

    //Discard Low Intensity and Low Gradient pixels
    void DLowIntGrad(QImage* src, QImage* dst);

};

#endif // HMTRACKINGMODULE_H
