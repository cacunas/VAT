#ifndef DEGUMODELINSTAnce_H
#define DEGUMODELINSTAnce_H


#include <QDomNode>
#include <QImage>
#include <vector>
#include <QSharedPointer>

#include "reliabilitysinglemodelinterface.h"
#include "src/blob.h"
#include "Blob2DFromBGSubstractionModel.h"

#include <opencv/cv.h>

class Datapool;

class DeguModelInstance: public Blob2DFromBGSubstractionModelInstance {
public:
    DeguModelInstance(Datapool *i_data);

    ~DeguModelInstance();

    //Initializes the visible model attributes for the interface.
    //The function is called by the interface constructor
    void initAttributes();

    //sets activation criteria: reliability on input (distance, bad data),
    //   needs (occlusion, priority),
    //sets priority of models (hierarchy).
    void setParameters(QDomNode &i_parameters);

    //sets activation flags for each model, according to the current context
    void activate(Blob *);

    //updates the model
    void update(Blob *);

    //sets global probability for the model (how good is the model)
    void setProbability();

    //sets global reliability for the model (how good is the data used in the model)
    void setReliability();

    //sets blob enclosing the model.
    void setBlobInterface();

    //sets global probability for the multi-model (how good is the data used in the model)
    Blob getBlobInterface();

    //Visualization

    void printAttributes();

    void printGeneral();

    bool draw(QImage *image);

    //Link to datapool
    //Datapool *m_data;

    //Flag for activation according to different criteria
    bool active;

    //Global probability and reliability for model
    double P, R;

    float Hues[360];

    std::vector<uchar> pixels;


    static int getContourToLineIntersection(std::vector<cv::Point> &hull,
                                             cv::Point3f &rline,
                                             cv::Point2f &r1, cv::Point2f &r2,
                                             int *n1 = NULL, int *n2 = NULL);

    //Fills a segments vector with moving pixel bounds for each normal on the axis
    void getNormalIntersections(cv::Mat &f, cv::Rect &roi, std::vector<cv::Point> &hull,
                                cv::Point2f &r1, cv::Point2f &r2, int n1, int n2,
                                float dx, float dy, std::vector< segment2D<float> > &segs,
                                std::vector< segment2D<float> > &hull_segs);

    static bool getGeneralLineForm(cv::Point2f p1, cv::Point2f p2,
                                        cv::Point3f &rline);
    static bool getGeneralLineForm(float x, float y, float dx, float dy,
                                   cv::Point3f &rline);


    //Attributes


    void checkHistory();

    void makeHistoryGraph();

};




#endif // DEGUMODELINSTAnce_H
