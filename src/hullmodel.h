#ifndef HULLMODEL_H
#define HULLMODEL_H

#include <opencv2/opencv.hpp>
#include <QSharedPointer>
#include "geometric.h"

class HullModel;

typedef QSharedPointer<HullModel> SpHullModel;

class HullModel {
public:
    HullModel();
    HullModel(HullModel &h);
    HullModel(SpHullModel h);
    HullModel(HullModel &h, int off_x, int off_y);
    ~HullModel();
    void setOffset(int, int);
    void setDistanceVector(SceneModel *s);

    //attributes
    unsigned long id;
    int off_x;
    int off_y;
    std::vector<cv::Point> local_hull;
    std::vector<cv::Point> hull;
    cv::Point axis1;
    cv::Point axis2;
    //Normal segments to distance
    std::vector< segment2D<float> > segs;
    //Normal segments to hull
    std::vector< segment2D<float> > hull_segs;
    //Length of normal segments to movement
    std::vector<double> dists;
    //Length of normal segments to hull
    std::vector<double> hull_dists;
};


#endif // HULLMODEL_H
