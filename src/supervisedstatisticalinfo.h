#ifndef SUPERVISEDSTATISTICALINFO_H
#define SUPERVISEDSTATISTICALINFO_H
#include<QSharedPointer>
#include <opencv2/opencv.hpp>
#include <QImage>

//Stores value and reliability data, associated to an output value
//obtained statistically from a sample
class SupervisedStatisticalInfo {
public:
    SupervisedStatisticalInfo();
    ~SupervisedStatisticalInfo();
    SupervisedStatisticalInfo(float m, float sd, float M, float r);
    void init(float mean, float sd, float median, float R);

    float mean;
    float sd;
    float median;
    float R;
    //Image associated to data. If not available, Image.isNull() is true.
    QImage *Image;
    cv::PCA *pca;
    std::vector<float> descriptor;
};

std::ostream& operator<<(std::ostream&, const SupervisedStatisticalInfo&);

#endif // SUPERVISEDSTATISTICALINFO_H
