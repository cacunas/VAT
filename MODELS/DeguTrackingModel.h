#ifndef DEGUTRACKINGMODEL_H
#define DEGUTRACKINGMODEL_H

#include "Blob2DFromBGSubstractionModel.h"
#include <opencv2/opencv.hpp>

#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilitysinglemodelinterface.h"
#include "src/blob.h"
#include "src/hullmodel.h"

class Datapool;

class DeguTrackingModel: public Blob2DFromBGSubstractionModel
{
public:
    DeguTrackingModel();

    ~DeguTrackingModel();

    void initDynamicsAttributes();

    //sets activation criteria for each model: reliability on input (distance, bad data),
    //   needs (occlusion, priority),
    //sets priority of models (hierarchy).
    void setParameters(QDomNode &i_parameters);




    //updates every activated model dynamics
    void updateDynamics();


    //sets blob enclosing the model.
    void setBlobInterface();


    void initAttribute(ReliabilityDynamicsAttribute &d, AttributeDupletValue &i);

    void copy(SpReliabilitySingleModelInterface);


    void makeHistoryGraph();
    void makeHistoryGraph_Dynamics();

    void checkHistory();
    void checkHistory_Dynamics();

    bool shapeAnalysis();
    bool shapeAnalysis_Dynamics();

    float Hues[360];

    std::vector<uchar> pixels;

    SpHullModel deguShape;

    //int Hues[36];

    int pSize;
    int pDiscard;
    int nFrames;

    float colorAccuracy;

    int m_first_color_range;
    int m_second_color_range;


    static bool movementFound(cv::Mat f, int wsize, int i0, int j0);
    static bool movementFound(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi);
    static double histogramDistance(cv::MatND h1, cv::MatND h2);
    static double distanceToSegment(int i1, int i2, std::vector<cv::Point> &contour);
    static bool lineSegmentIntersection(cv::Point3f &line,
                                 cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &r);
    static bool getGeneralLineForm(cv::Point2f p1, cv::Point2f p2,
                                        cv::Point3f &rline);
    static bool getGeneralLineForm(float x, float y, float dx, float dy,
                                   cv::Point3f &rline);


    //Get intersection points r1 and r2 between line 'rline' and contour 'hull' (oriented to
    //convex hull). If non NULL n1 and n2 pointers, are used to store indexes of contour segments
    //where intersections were found (to be used later on pixel distance function construction).
    static int getContourToLineIntersection(std::vector<cv::Point> &hull,
                                             cv::Point3f &rline,
                                             cv::Point2f &r1, cv::Point2f &r2,
                                             int *n1 = NULL, int *n2 = NULL);

    //Fills a segments vector with moving pixel bounds for each normal on the axis
    void getNormalIntersections(cv::Mat &f, cv::Rect &roi, std::vector<cv::Point> &hull,
                                cv::Point2f &r1, cv::Point2f &r2, int n1, int n2,
                                float dx, float dy, std::vector< segment2D<float> > &segs,
                                std::vector< segment2D<float> > &hull_segs);

    //Sets the moving segment for one normal
    void setForegroundSegment(cv::Mat &f, cv::Rect &roi, std::vector<cv::Point> &hull,
                              segment2D<float> &seg, segment2D<float> &hseg, float x, float y,
                              float dx, float dy, int &I1, int &I2);


    //Gets the intersection betweel a line and a contour, in current or next contour
    //segment.
    static int getContourToLineIntersectionIndexed(std::vector<cv::Point> &polygon,
                                            cv::Point3f &rline,
                                             cv::Point2f &r1, cv::Point2f &r2,
                                             int &n1, int &n2);

    //Counts foreground pixels on a window
    static int movementCount(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi);

    void findHead(SpHullModel hull);


    cv::Point2f headLocation;
};

//typedef QSharedPointer<Blob2DFromBGSubstractionModelDynamics> SpBlob2DFromBGSubstractionModelDynamics;

#endif // DEGUTRACKINGMODEL_H
