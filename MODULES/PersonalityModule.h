#ifndef PERSONALITYMODULE_H
#define PERSONALITYMODULE_H

#include <map>
#include <string>
#include <QImage>
#include "Datapool.h"
#include "QWT/qwt_polar_plot.h"
#include "ModuleInterface.h"
#include "src/hullmodel.h"
#include "src/supervisedstatisticalinfo.h"

#include <QDir>

#define  M_1_DIV_2PI 0.159154943091895


class PersonalityModule: public ModuleInterface {
public:
    PersonalityModule(Datapool *i_data);
    ~PersonalityModule();

    //Set module configuration parameters
    bool setParameters(QDomNode& config);

    //Initialization after reading parameters
    bool init();

    //Function executed at each frame
    bool run();

    //update parameters at runtime.
    bool updateParameters();

    //Reads file formated
    int readSupervisedData();

    void getPersonality(QImage *current, SupervisedStatisticalInfo &sup);

    //For parameter tuning, separet procedure
    void getPersonalityPreAngular(QImage *current,
                                  std::vector< std::vector<cv::Point> > &ccontours);
    void getPersonalityFromAngular(std::vector< std::vector<cv::Point> > &contours,
                                   SupervisedStatisticalInfo &sup);



    static void getFarPoints(cv::Mat im, float dx, float dy, cv::Point2f &r1, cv::Point2f &r2, float mean_x, float mean_y);
    static bool pointInNeighborhood(cv::Mat im, float pos_x, float pos_y,
                             int &x, int &y);
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
                                float dx, float dy, std::vector< segment2D<float> > &segs);

    //Sets the moving segment for one normal
    void setForegroundSegment(cv::Mat &f, cv::Rect &roi, std::vector<cv::Point> &hull,
                              segment2D<float> &seg, float x, float y,
                              float dx, float dy, int &I1, int &I2);

    //Gets the intersection betweel a line and a contour, in current or next contour
    //segment.
    static int getContourToLineIntersectionIndexed(std::vector<cv::Point> &polygon,
                                            cv::Point3f &rline,
                                             cv::Point2f &r1, cv::Point2f &r2,
                                             int &n1, int &n2);

    //Counts foreground pixels on a window
    static int movementCount(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi);

    //Copies a QImage to a cv::Mat, considering the possible difference between
    //bytes per line between them.
    static void safeCopyQImageToMat(QImage &q, cv::Mat &m);

    //Draw contour to mat colouring it from blue to red
    static void drawContourToMat(std::vector<cv::Point> &v, cv::Mat &m);

    //Draw contour to mat with a given color
    static void drawContourToMat(std::vector<cv::Point> &v, cv::Mat &m, cv::Scalar color);

    //Gaussian blur for a contour
    static void blurContour(std::vector<cv::Point> &contour,
                                        int scale, int step);

    //Connect contours by nearest extremes
    void connectContours(std::vector< std::vector<cv::Point> > &contours,
                         std::vector< std::vector<cv::Point> > &ccontours);

    //Filter short contours
    void filterContours(std::vector< std::vector<cv::Point> > &contours,
                        std::vector< std::vector<cv::Point> > &fcontours,
                        int min_length);

    //Draw points
    void drawPointsToMat(cv::vector<cv::Point2f> &pca_points, cv::Mat &m);

    void drawKMeansPointsToMat(cv::vector<cv::Point2f> &pca_points, cv::Mat &m,
                         cv::Mat &klabels, cv::Mat &kcenters, int K);


    //The process
    //1. Prepare current image
    void prepareImage(cv::Mat &current);

    //2. Get Shape Border
    void getShapeBorder(cv::Mat &current, cv::Mat &border);

    //3. Get Centroid. Returns the number of considered points
    int getCentroid(cv::Mat &image, float x, float y);

    //4. Get Main Axis
    void getMainAxis(cv::Mat &image, float x, float y, int n,
                     cv::Point2f &r1, cv::Point2f &r2);

    //8.1 Get Angular Function
    void getAngularFunction(std::vector<float> &angular,
                            std::vector< std::vector<cv::Point> > &contours);

    //8.2 Get Histogram
    void getHistogram(std::vector<float> &input, cv::MatND &hist);

    cv::PCA *getPCAfromHistograms(std::vector<cv::MatND> h);
    cv::PCA *getPCAfromHistograms(std::vector<cv::MatND> h, cv::Mat &pcadata, cv::Mat &pcamean);

    cv::PCA *getPCA(cv::Mat &pcadata, cv::Mat &pcamean);

    //Returns kcenters from KMeans por pca eigenvectors
    cv::Mat getKMeansFromPCA(cv::PCA *pca, cv::Mat &pcadata, cv::Mat &pcamean);

    float crossValidation(std::vector<SupervisedStatisticalInfo> &supervised, std::vector<CvANN_MLP> &anns, float &rel_error);

    CvANN_MLP getModel(std::vector<SupervisedStatisticalInfo> &supervised,
                       bool mean, bool reliability);
    void getModel(std::vector<SupervisedStatisticalInfo> &sup,
                  CvANN_MLP &ann, bool mean, bool reliability);

    float getPrediction(CvANN_MLP &ann,
                        SupervisedStatisticalInfo &sup);


    int line;
    int cont;

    int m_min_images_to_process;
    int processed_images;
    int m_bins;
    int w_size;
    int m_hullOffset;
    bool m_abs_angular_diff;
    float m_K;

    int m_num_ev; //num of eigenvectors to consider in K-means
    int m_clusters; //kmeans clusters
    cv::Mat kcenters; //last result, k-means centers
    bool m_use_mean;
    bool m_use_reliability;

    std::vector<cv::MatND> histograms;
    std::vector<SupervisedStatisticalInfo> supervised;
    std::vector<CvANN_MLP> anns;
    //For tuning:
    std::vector< std::vector< std::vector<cv::Point> > > image_contours;

    std::map<int, double> gaussianConstant;
    std::map<int, double> gaussianFactor;

    std::vector<std::string> names;
    std::vector<bool> elegant;

    QwtPolarPlot polarPlot;

    std::string m_supervisedFile;
    int m_min_contour_length;
    bool first;
    int m_scale;
    int m_ostep;
    bool m_tuning_mode;

};

std::ostream& operator<<(std::ostream& out, CvANN_MLP &ann);

#endif // PERSONALITYMODULE_H
