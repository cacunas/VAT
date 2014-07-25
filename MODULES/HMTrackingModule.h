#ifndef HMTRACKINGMODULE_H
#define HMTRACKINGMODULE_H

#include <QImage>
#include <QDir>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>
#include <vector>
#include <cmath>
#include <QColor>

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

    static cv::Mat qImage2Mat(QImage * qImage)
    {
    //	int width = qImage->width();
        int height = qImage->height();

        void *raw = qImage->bits();
        int bytesPerLine = qImage->bytesPerLine();

        /*!
        // Copiando sería así:
    //	cv::Mat f(height, bytesPerLine, CV_8U);
    //	memcpy(f.data, raw, height*bytesPerLine);
        // Y después habría que actualizar la imagen inicial así:
    //	int height = fg->height();
    //	int bytesPerLine = fg->bytesPerLine();
    //	uchar * bits = fg->bits();
    //	memcpy(bits, image.data, height*bytesPerLine);
    */

        // Pero yo lo hago chantamente linkeandolo con la representación del QImage
        cv::Mat f(height, bytesPerLine, CV_8U, raw);
        return f;
    }

private:
    bool firstTime;
    float alpha, beta;
    float A_p[4];//, G_p, B_p, GL_p; // histogram intensity peaks
    float A_t[4];//, G_t, B_t, GL_t; // intensity thresholds
    //float R_u, G_u, B_u, GL_u; // histogram intensity mean
    //float R_std, G_std, B_std, GL_std; //histogram standar deviation


    //Convert QImage to cv::Mat
    cv::Mat qimage_to_mat_cpy(QImage* img, int format);

    //Convert cv::Mat to QImage
    QImage* Mat2QImage(cv::Mat& mat);

    //Calculates Intensity Histograms per channel
    vector<hist> calculateHistograms(QImage* img);

    //Calculates first and second statistical moments of the intesity histograms
    vector<float> calculateMoments(vector<hist> channels);

    //Calculate channels peaks using paper proposed criterias
    void calculatePeaks(vector<hist> channels);

    //calculate thresholds
    void calculateThresholds(vector<hist> ch);

    //Store grass area of the image on bgImage using a binary classifier
    void GrassClassifier();

    //Applies convolution with the 3x3 h filter, so it can detects line marks on field
    void HFilter(QImage *img);

};

#endif // HMTRACKINGMODULE_H
