#ifndef HULLTOOBJECTMODULE_H
#define HULLTOOBJECTMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "ModuleInterface.h"
#include "src/hullmodel.h"

#include <QDir>

class HullToObjectModule: public ModuleInterface {
public:
    HullToObjectModule(Datapool *i_data);
    ~HullToObjectModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    void completeBlobs(std::deque<SpRMMMobileObject> &objects, QImage *fg, QImage *current);
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

    int line;
    int cont;

    int m_bins;
    int w_size;
    int m_hullOffset;
    float m_K;
};

#endif // HULLTOOBJECTMODULE_H
