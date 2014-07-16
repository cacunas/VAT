#ifndef HMTRACKINGMODULE_H
#define HMTRACKINGMODULE_H

#include <QImage>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cmath>
#include <QColor>

#include "Datapool.h"
#include "ModuleInterface.h"

typedef std::vector<int> hist;

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
    float R_p, G_p, B_p, GL_p; // histogram intensity peaks
    float R_t, G_t, B_t, GL_t; // intensity thresholds
    float R_u, G_u, B_u, GL_u; // histogram intensity mean
    float R_std, G_std, B_std, GL_std; //histogram standar deviation


    //Convert QImage to cv::Mat
    cv::Mat qimage_to_mat_cpy(QImage* img, int format);

    //Store grass area of the image on bgImage using a binary classifier
    void GrassClassifier();

    //Calculate channels peaks using paper proposed criterias
    void calculatePeaks(hist r, hist g, hist b, hist gray);
};

#endif // HMTRACKINGMODULE_H
