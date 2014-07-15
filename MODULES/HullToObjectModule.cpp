#include "HullToObjectModule.h"
#include "image_display.h"
#include "src/hullmodel.h"
#include <errno.h>
#include <iostream>
#include <QMessageBox>
#include <QPainter>
#include <deque>
#include <src/blob.h>
#include <map>
#include <set>
#include <QFile>
#include <QTextStream>

//cv::RNG rng(12345);

#define RSEG_DEBUG

const int maxComponents = 1;

HullToObjectModule::HullToObjectModule(Datapool *i_data): ModuleInterface(i_data) {
    m_bins = 32;
    w_size = 11;
    m_hullOffset = 1;
    m_K = 2;
}

HullToObjectModule::~HullToObjectModule() {}

bool HullToObjectModule::setParameters(QDomNode& config){
    QDomNode n;

    if(config.isNull()) { //Parameter set for module not defined
        m_bins = 32;
        w_size = 11;
        m_hullOffset = 1;
        m_K = 2;
    } else {
        if( !( n = XmlCommon::getParameterNode("ColorHistogramBinSize", config) ).isNull() ) {
            m_bins = XmlCommon::getParameterValue(n).toInt();
            if(m_bins <= 0 || m_bins > 128) {
                m_bins = 32;
                AppendToLog("HullToObjectModule: Warning: 'ColorHistogramBinSize' must be in ]0,128]. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(m_bins) );
            }
        } else { //default
            m_bins = 32;
            AppendToLog("HullToObjectModule: Warning: 'ColorHistogramBinSize' not defined. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(m_bins) );
        }
        addParameter("ColorHistogramBinSize", QString::number(m_bins), "int");

        if( !( n = XmlCommon::getParameterNode("SearchWindowSize", config) ).isNull() ) {
            w_size = XmlCommon::getParameterValue(n).toInt();
            if(w_size <= 0) {
                w_size = 11;
                AppendToLog("HullToObjectModule: Warning: 'SearchWindowSize' must be higher than 0. Taking default: \n\t\tColorHistogramBinSize = " + QString::number(w_size) );
            }
        } else { //default
            w_size = 11;
            AppendToLog("HullToObjectModule: Warning: 'SearchWindowSize' not defined. Taking default: \n\t\tSearchWindowSize = " + QString::number(w_size) );
        }
        addParameter("SearchWindowSize", QString::number(w_size), "int");

        if( !( n = XmlCommon::getParameterNode("ConvexHullOffset", config) ).isNull() ) {
            m_hullOffset = XmlCommon::getParameterValue(n).toInt();
        } else { //default
            m_hullOffset = 1;
            AppendToLog("HullToObjectModule: Warning: 'ConvexHullOffset' not defined. Taking default: \n\t\tConvexHullOffset = " + QString::number(m_hullOffset) );
        }
        addParameter("ConvexHullOffset", QString::number(m_hullOffset), "int");

        if( !( n = XmlCommon::getParameterNode("PrincipalAxisStep", config) ).isNull() ) {
            m_K = XmlCommon::getParameterValue(n).toInt();
        } else { //default
            m_K = 2;
            AppendToLog("HullToObjectModule: Warning: 'PrincipalAxisStep' not defined. Taking default: \n\t\tPrincipalAxisStep = " + QString::number(m_K) );
        }
        addParameter("PrincipalAxisStep", QString::number(m_K), "int");
    }

    return true;
}

bool HullToObjectModule::updateParameters(){
    parameter *bins, *size, *hull;
    bins = getParameter("ColorHistogramBinSize");
    size = getParameter("SearchWindowSize");
    hull = getParameter("ConvexHullOffset");

    if( bins == 0 || size == 0 || hull == 0 ||
            bins->value.toInt() <= 0 || bins->value.toInt() > 128 ||
            size->value.toInt() <= 0 )
            return false;
    return true;
}

bool HullToObjectModule::init(){
    return true;
}

//Formato ARGB
//b == bits[i]    g == bits[i+1]    r == bits[i+2]    alpha == bits[i+3]
bool HullToObjectModule::run(){
    if(m_data->hulls.size() > 0)
        m_data->hulls.clear();
    if(m_data->RMMobjects.size() > 0 && m_data->fgImage != NULL && m_data->currentImage != NULL) //Requires foreground and current images
        completeBlobs(m_data->RMMobjects, m_data->fgImage, m_data->currentImage);

    return true;
}

void HullToObjectModule::completeBlobs(std::deque<SpRMMMobileObject> &objects,
                                               QImage *fg, QImage *current) {

    if(objects.size() == 0)
        return;
    int nbins = 256/m_bins;
    int i, j, w = fg->width(), h = fg->height(), k=0;
    int i0, j0, w0, h0;
    double maxVal;

    QImage curr888 = current->convertToFormat(QImage::Format_RGB888);
    cv::Mat c(h, w, CV_8UC3), c_yuv(h, w, CV_8UC3),
            f(h, w, CV_8UC1), f0(h, w, CV_8UC1), r(h, w, CV_8UC3);

    int bl = fg->bytesPerLine(), bl2 = curr888.bytesPerLine();
    std::deque<SpRMMMobileObject>::iterator it, it_end = objects.end();
    uchar d1, d2, d3,
          *fg_p = fg->bits(),
          *c_p = curr888.bits();

    memcpy(c.data, c_p, h*bl2);
    memcpy(f.data, fg_p, h*bl);
    memset(c_yuv.data, 0, h*bl2);
    memset(r.data, 0, h*bl2);

    f.copyTo(f0);

    cv::Rect roi;
    //Histogram parameters
    int channels[] = {1, 2};
    int histSize[] = {nbins, nbins};
    float pranges[] = { 0, 256 };
    const float* ranges[] = { pranges, pranges };

    //Rectangular structuring element
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                           cv::Size( 3, 3 ),
                                           cv::Point( 1, 1 ) );

    //Set local window pixel histogram for comparison
    cv::Rect wroi;
    wroi.width = wroi.height = w_size/2;

    //Start blobs processing for hulls calculation
    for(it=objects.begin(); it!=it_end; it++) {
        k++;

        SpRMMMobileObject obj = (*it);
        SpHullModel newHull(new HullModel());
        Blob &b = obj->multiModel.binterface;
        i0 = b.bbox.ytop, j0 = b.bbox.xleft,
        h0 = b.bbox.height, w0 = b.bbox.width;
        if(j0 >= f.cols || i0 >= f.rows || h0 <= 0 || w0 <= 0) {
            m_data->hulls.push_back(newHull);
            continue;
        }        

        if(j0 < 0) {
            w0 += j0;
            j0 = 0;
            if(w0 <= 0) w0 = 1;
        }

        if(i0 < 0) {
            h0 += i0;
            i0 = 0;
            if(h0 <= 0) h0 = 1;
        }

        if(j0 + w0 > f.cols)
            w0 = f.cols - j0;

        if(i0 + h0 > f.rows)
            h0 = f.rows - i0;

        //Con la misma Mat f .....

        //Apertura en blob
        roi.x = j0;
        roi.y = i0;
        roi.width = w0;
        roi.height = h0;


/*        if(m_data->frameNumber == 962) {
            std::cout << "Error frame " << m_data->frameNumber << std::endl;
            std::cout << "Mobile id " << obj->mobile_id << std::endl;
            std::cout << "\troi.x: " << roi.x;
            std::cout << "\troi.y: " << roi.y;
            std::cout << ";\troi.width: " << roi.width;
            std::cout << ";\troi.height: " << roi.height;
            std::cout << ";\tf.cols: " << f.cols << std::endl;
        } */


        //Restrict operations to blob zone

        if(!(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= f.cols)) {
            std::cout << "Error frame " << m_data->frameNumber << std::endl;
            std::cout << "\troi.x: " << roi.x;
            std::cout << ";\troi.width: " << roi.width;
            std::cout << ";\tf.cols: " << f.cols << std::endl;
        }

        if(!(0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= f.rows)) {
            std::cout << "Error frame " << m_data->frameNumber << std::endl;
            std::cout << "\troi.x: " << roi.y;
            std::cout << ";\troi.width: " << roi.height;
            std::cout << ";\tf.cols: " << f.rows << std::endl;
        }

        cv::Mat aux(f, roi);
        cv::Mat aux0(f0, roi);

        //Reduce bad detections, in general near borders
        cv::erode(aux, aux, element, cv::Point(-1,-1), 1);

        //Reduce bad detections, in general near borders and recover shape
        cv::erode(aux0, aux0, element, cv::Point(-1,-1), 1);
        cv::dilate(aux0, aux0, element, cv::Point(-1,-1), 1);

        //Border detection
        cv::Mat border_aux(aux.size(), CV_8UC1);
        cv::Canny(aux,border_aux, 50,100, 3);

#ifdef RSEG_DEBUG
//        cv::namedWindow( "Canny", 1 );
//        cv::imshow( "Canny", border_aux );
#endif

        //Find confining convex hull (Note: used border_copy as findContours modifies the image)
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat border_copy(border_aux.size(), CV_8UC1);
        border_aux.copyTo(border_copy);
        cv::findContours(border_copy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

#ifdef RSEG_DEBUG
/*        cv::Scalar color = cv::Scalar( 255, 255, 255);
        cv::Mat drawing = cv::Mat::zeros( border_aux.size(), CV_8UC3);
        for(i = 0; i< contours.size(); i++ )
            cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
        cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        cv::imshow( "Contours", drawing );*/
#endif

        //One contour to confine all detected contours
        std::vector<cv::Point> big_contour;
        std::vector<cv::Point> hull;
        if(contours.size() > 0) {
            //Group found contours in one big contour
            for(i=0;i<contours.size(); i++) {
                if(hierarchy[i][2] < 0) { // No parent, so it's parent
                    if(big_contour.empty())
                        big_contour = contours[i];
                    else
                        big_contour.insert( big_contour.end(), contours[i].begin(), contours[i].end());
                }
            }
            //Get initial convex hull
            cv::convexHull( big_contour, hull, false );

#ifdef RSEG_DEBUG
            //Print contour and hull
            /*std::cout << "Hull" << std::endl;
            for(i=0; i<hull.size(); i++)
                std::cout << hull[i].x << "," << hull[i].y << std::endl;
            cv::Mat drawing2 = cv::Mat::zeros( border_aux.size(), CV_8UC3);
            cv::Scalar color = cv::Scalar( 255, 0, 255 );
            std::vector<std::vector<cv::Point> > drawc, drawh;
            drawc.push_back(big_contour);
            drawh.push_back(hull);
            color = cv::Scalar( 0, 0, 255 );
            cv::drawContours( drawing2, drawh, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            color = cv::Scalar( 255, 0, 255 );
            cv::drawContours( drawing2, drawc, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
            cv::namedWindow( "Contour and Hull", CV_WINDOW_AUTOSIZE );
            cv::imshow( "Contour and Hull", drawing2 );*/
#endif
        } else {
            m_data->hulls.push_back(newHull);
            continue;
        }
        if(hull.size() == 0) {
            m_data->hulls.push_back(newHull);
            continue;
        }

        //Confine current image to blob, and get inverted foreground mask
        cv::Mat caux(c, roi), aux2 = 255 - aux;

        //COLOR HISTOGRAM
        //Get YCrCb image
        cv::Mat c_yuvaux(c_yuv, roi);
        cv::cvtColor(caux, c_yuvaux, CV_BGR2YCrCb);

        //Calculate foreground and background chroma histograms
        cv::MatND hist, hist2;
        //Foreground
        cv::calcHist( &c_yuvaux, 1, channels, aux, // do not use mask
                  hist, 2, histSize, ranges,
                 true, // the histogram is uniform
                 false );
        maxVal=0;
        cv::minMaxLoc(hist, 0, &maxVal, 0, 0);
        hist = hist/maxVal;
        //Background
        cv::calcHist( &c_yuvaux, 1, channels, aux2, // do not use mask
                    hist2, 2, histSize, ranges,
                   true, // the histogram is uniform
                   false );
        maxVal=0;
        cv::minMaxLoc(hist2, 0, &maxVal, 0, 0);
        hist2 = hist2/maxVal;

        //Check correlation between color histograms:
        cv::MatND pixhist;
        for(i = i0; i < i0 + h0; i++ ) {
            for(j = j0; j < j0 + w0; j++ ) {
                //Just for points inside the convex hull and a little offset
                if(cv::pointPolygonTest(hull, cv::Point2f(j-j0,i-i0), true) > - m_hullOffset) {
                    if(f.data[i*bl+j]) { //Movement point
                       //Set augmented segmentation image
                       r.data[i*bl2+3*j] = r.data[i*bl2+3*j+1] = r.data[i*bl2+3*j+2] = 255; //White
                    } else { //Non-movement
                        //Check neighborhood for movement.
                       if(    j + w_size/2 >= w || i + w_size/2 >= h
                           || j - w_size/2 < 0  || i - w_size/2 < 0 )
                            continue;
                        wroi.x = j - w_size/2;
                        wroi.y = i - w_size/2;
                        if(movementFound(f, w_size, i, j, roi)) {
                            //Generate local histogram for comparison
                            cv::Mat c_yuvpix(c_yuv, wroi);
                            cv::calcHist( &c_yuvpix, 1, channels, cv::Mat(), // do not use mask
                                        pixhist, 2, histSize, ranges,
                                        true, // the histogram is uniform
                                        false );
                            maxVal = 0;
                            cv::minMaxLoc(pixhist, 0, &maxVal, 0, 0);
                            pixhist = pixhist/maxVal;

                            //Decide if background or foreground, comparing histograms
                            if(histogramDistance(hist,pixhist) < histogramDistance(hist2,pixhist)) {
                                r.data[i*bl2+3*j] = 255; //Red
                            }
                        }
                    }
                }
            }
        }
        //Integrate results with original mask
        for(i = i0; i < i0 + h0; i++ )
            for(j = j0; j < j0 + w0; j++ )
                if(f0.data[i*bl+j] != 0 || r.data[i*bl2+3*j] != 0 || r.data[i*bl2+3*j+1] != 0 || r.data[i*bl2+3*j+2] != 0) {
                    f.data[i*bl+j] = 255;
                    if(f0.data[i*bl+j] != 0)
                        r.data[i*bl2+3*j] = r.data[i*bl2+3*j+1] = r.data[i*bl2+3*j+2] = 255;
                }
        //Opening and Closing
        cv::erode(aux, aux, element);
        cv::dilate(aux, aux, element,cv::Point(-1,-1),2);
        cv::erode(aux, aux, element);

        //Recalculate Convex Hull
        cv::Canny(aux,border_aux, 50,100, 3);
        contours.clear();
        hierarchy.clear();
        big_contour.clear();
        hull.clear();
        cv::findContours(border_aux, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        for(i=0;i<contours.size(); i++) {
            if(hierarchy[i][2] < 0) { // No parent, so it's parent
                if(big_contour.empty())
                    big_contour = contours[i];
                else
                    big_contour.insert( big_contour.end(), contours[i].begin(), contours[i].end());
            }
        }
        cv::convexHull( big_contour, hull, false );

        newHull->local_hull = hull;
        newHull->off_x = j0;
        newHull->off_y = i0;
        newHull->id = (*it)->mobile_id;

        //Get principal/minor axis
        std::vector<cv::Point2f> data_aux(h0*w0);
        float mean_x = 0, mean_y = 0;
        int count = 0;

        for(i=0; i<h0; i++)
            for(j=0; j<w0; j++)
                if(cv::pointPolygonTest(hull, cv::Point2f(j, i), true) > - m_hullOffset) {
                    data_aux[count++] = cv::Point2f(j, i);
                    mean_x += j;
                    mean_y += i;
                }
        //data_aux.resize(count);
        //cv::Mat data(2, count, CV_32FC1, &data_aux.front());
        cv::Mat data(2, count, CV_32FC1);
        cv::Point2f x;
        for(i=0; i<count; i++) {
            data.at<float>(0,i) = data_aux[i].x;
            data.at<float>(1,i) = data_aux[i].y;
        }

        //cv::Mat data();
        mean_x /= count;
        mean_y /= count;
        cv::Mat mean(2, 1, CV_32FC1);
        mean.at<float>(0) = mean_x;
        mean.at<float>(1) = mean_y;

        //2. perform PCA
        cv::PCA pca(data, mean, CV_PCA_DATA_AS_COL, maxComponents);
        //result is contained in pca.eigenvectors (as row vectors)
        //std::cout << pca.eigenvectors << std::endl;

        //3. get angle of principal axis
        float dx = pca.eigenvectors.at<float>(0, 0),
              dy = pca.eigenvectors.at<float>(0, 1),
              scale = 40.0;
        cv::Point3f rline;
        cv::Point2f r1, r2;

        //Get line general form from principal component
        getGeneralLineForm(cv::Point2f(mean_x, mean_y),
                           cv::Point2f(mean_x + dx*scale, mean_y + dy*scale),
                           rline);
        //Get segment from line
        int n1, n2;
        getContourToLineIntersection(hull, rline, r1, r2, &n1, &n2);

        //Get pixel intersections for normals
        std::vector< segment2D<float> > &segs = newHull->segs;
        std::vector< segment2D<float> > &hull_segs = newHull->hull_segs;

        //Get segments of movement normal to principal axis. Also reorders r1 and r2 in
        //coherence with segments order
        getNormalIntersections(aux, roi, hull, r1, r2, n1, n2, dx, dy, segs, hull_segs);

        newHull->axis1 = r1;
        newHull->axis2 = r2;


        //Set new representation
        m_data->hulls.push_back(newHull);

        //Get the pixel distance function
        std::vector<float> dfunction;
        //dfunction.resize((int)D_axis + 1); //

                //First and last are zero for sure (axis intersects contour).
        //dfunction[0] = 0.0;
        //dfunction[(int)D_axis] = 0.0;
        //for

#ifdef RSEG_DEBUG
        /*std::cout << "Final Hull" << std::endl;
        for(i=0; i<hull.size(); i++)
            std::cout << i << " : " << hull[i].x << " ; " << hull[i].y << std::endl;
*/
/*        std::cout << "Distances" << std::endl;
        for(i=0; i<segs.size(); i++) {
            double dx = segs[i].first.x - segs[i].last.x;
            double dy = segs[i].first.y - segs[i].last.y;
            std::cout << i << " : " << sqrt(dx*dx+dy*dy) << std::endl;
        }
        color = cv::Scalar( 0, 255, 255 );
        std::vector<std::vector<cv::Point> > drawc;
        drawc.push_back(hull);
        cv::Mat raux(r, roi);
        cv::drawContours( raux, drawc, 0, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        color = cv::Scalar( 0, 255, 0 );
        cv::line(raux, r1, r2, color);
        cv::line(raux, cv::Point(mean_x - dx*scale, mean_y - dy*scale),
                       cv::Point(mean_x + dx*scale, mean_y + dy*scale), color);
        cv::namedWindow( "Final", CV_WINDOW_AUTOSIZE );
        cv::imshow( "Final", raux );*/
#endif
    }
    //Set datapool images
    memcpy(fg_p, f.data, h*bl);
    memcpy(m_data->rFgImage->bits(), r.data, h*bl2);

}

void HullToObjectModule::getNormalIntersections(cv::Mat &f,
                                   cv::Rect &roi,
                                   std::vector<cv::Point> &hull,
                                   cv::Point2f &r1, cv::Point2f &r2,
                                   int n1, int n2,
                                   float dx, float dy,
                                   std::vector< segment2D<float> > &segs,
                                   std::vector< segment2D<float> > &hull_segs) {
//                                   std::vector< cv::Point2f > &) {
    cv::Point2f R1, R2;
    int I1, I2, sy;
    //Normalize search: from leftmost point, if equal, higher one.
    if(r1.x < r2.x) {
        R1 = r1;
        R2 = r2;
        I1 = n1;
        sy = (R1.y <= R2.y) ? 1 : -1;
    } else if(r1.x > r2.x) {
        R1 = r2;
        R2 = r1;
        I1 = n2;
        sy = (R1.y <= R2.y) ? 1 : -1;
    } else {
        sy = 1;
        if(r1.y <= r2.y) {
            R1 = r1;
            R2 = r2;
            I1 = n1;
        } else {
            R1 = r2;
            R2 = r1;
            I1 = n2;
        }
    }
    I2 = I1;

    //Set scale and advances in X and Y to move K pixels, given dx and dy.
    float lx = R2.x - R1.x, ly = R2.y - R1.y,
          D_axis = sqrt(lx*lx + ly*ly),
          a = m_K/sqrt(dx*dx + dy*dy),
          Dx = fabs(a*dx),
          Dy = sy*fabs(a*dy);
    int i, n = (int)D_axis/m_K;
    float x, y;

    segs.resize(n);
    hull_segs.resize(n);


    for(i=0, x=R1.x + Dx, y=R1.y + Dy; i<n; i++, x+=Dx, y+=Dy)
        setForegroundSegment(f, roi, hull, segs[i], hull_segs[i], x, y, dx, dy, I1, I2);

    //Reorder principal axis points, w/r to normal segments
    r1 = R1;
    r2 = R2;
}


void HullToObjectModule::setForegroundSegment(cv::Mat &f, cv::Rect &roi,
                                                      std::vector<cv::Point> &hull,
                                                      segment2D<float> &seg,
                                                      segment2D<float> &hseg,
                                                      float x, float y,
                                                      float dx, float dy,
                                                      int &I1, int &I2) {
    cv::Point3f rline;
    //Get line form from perpendicular line in x,y
    getGeneralLineForm(x, y, dy, -dx, rline);

    cv::Point2f r1, r2;

//    if(getContourToLineIntersectionIndexed(hull, rline, r1, r2, I1, I2) == 2) {
    if(getContourToLineIntersection(hull, rline, r1, r2) == 2) {
        float xt, yt, Dx = r2.x - r1.x, Dy = r2.y - r1.y, D = sqrt(Dx*Dx + Dy*Dy);
        Dx /= D; Dy /= D; //Unitary increment
        int d, xx, yy, w_mean = 9/2;

        hseg.first.x = r1.x;
        hseg.first.y = r1.y;
        hseg.last.x = r2.x;
        hseg.last.y = r2.y;

        for(xt=r1.x, yt=r1.y, d=0; d<D; xt+=Dx, yt+=Dy, d++) {
            xx = rint(xt); yy = rint(yt);
            if(movementCount(f, 3, yy, xx, roi) > w_mean) {
                seg.first.x = xx;
                seg.first.y = yy;
                break;
            }
        }
        for(xt=r2.x, yt=r2.y, d=0; d<D; xt-=Dx, yt-=Dy, d++) {
            xx = rint(xt); yy = rint(yt);
            if(movementCount(f, 3, yy, xx, roi) > w_mean) {
                seg.last.x = xx;
                seg.last.y = yy;
                break;
            }
        }
    } else
        std::cout << "setForegroundSegment: Error: 2 intersections not found" << std::cout;

}

int HullToObjectModule::getContourToLineIntersectionIndexed(std::vector<cv::Point> &polygon,
                                         cv::Point3f &rline,
                                         cv::Point2f &r1, cv::Point2f &r2,
                                         int &n1, int &n2) {

    int i, k1 = 0, k2 = 0, n = polygon.size();
    cv::Point2f p1_1, p2_1, p1_2, p2_2, r;
    bool ready1 = false, ready2 = false, got_same = false;

    p1_1 = polygon[n1];
    p2_1 = polygon[(n1+1 == n ? 0 : n1+1)];
    if(lineSegmentIntersection(rline, p1_1, p2_1, r))
        got_same = true;
    n1 = (n1+1 == n ? 0 : n1+1);
    n2 = (n2-1 == -1) ? n-1 : n2-1;

    do {
        if(!ready1) {
            p1_1 = polygon[n1];
            p2_1 = polygon[(n1+1 == n ? 0 : n1+1)];
            if(lineSegmentIntersection(rline, p1_1, p2_1, r1)) {
                ready1 = true;
                if(got_same) {
                    r2 = r;
                    break;
                }
            } else
                n1 = (n1+1 == n ? 0 : n1+1);
            k1++;
        }
        if(!ready2) {
            p1_2 = polygon[n2];
            p2_2 = polygon[(n2+1 == n ? 0 : n2+1)];
            if(lineSegmentIntersection(rline, p1_2, p2_2, r2)) {
                ready2 = true;
                if(got_same) {
                    r1 = r;
                    break;
                }
            } else
                n2 = (n2-1 == -1) ? n-1 : n2-1;
            k2++;
        }
        if(k1 > n && k2 > n) //If true, there is a problem
            break;

    } while (!ready1 || !ready2);

    if(k1 > n && k2 > n) //If true, there is a problem
        return 0;
    else if(k1 > n || k2 > n) //If true, there is a problem
        return 1;
    return 2;
}

int HullToObjectModule::getContourToLineIntersection(std::vector<cv::Point> &polygon,
                                         cv::Point3f &rline,
                                         cv::Point2f &r1, cv::Point2f &r2,
                                         int *n1, int *n2) {
    int i, ind[3], n = polygon.size();
    int num_found = 0;
    cv::Point2f p1, p2, pts[3]; //Max 4 for a convex polygon (2 in 2 corners)
    p2 = polygon[n-1];
    for(i=0; i<n; i++) {
        p1 = p2;
        p2 = polygon[i];
        if(lineSegmentIntersection(rline, p1, p2, pts[num_found])) {
            ind[num_found] = i == 0 ? n-1 : i-1;
            if(++num_found == 3)
                break;
        }
    }

    if (num_found == 1) {
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[0];
        r1 = pts[0];
        return 1;
    } else if(num_found == 2) {
        r1 = pts[0];
        r2 = pts[1];
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[1];
        return 2;
    }

    //If more than two, the intersection repeated a polygon point
    if(   fabs(pts[0].y - pts[1].y) < /*EPS*/1e-8
       && fabs(pts[0].x - pts[1].x) < /*EPS*/1e-8) { //Take 1 and 3
        r1 = pts[0];
        r2 = pts[2];
        if(n1 != NULL)
            *n1 = ind[0];
        if(n2 != NULL)
            *n2 = ind[2];

        return 3;
    }

    r1 = pts[0];
    r2 = pts[1];
    if(n1 != NULL)
        *n1 = ind[0];
    if(n2 != NULL)
        *n2 = ind[1];

    return 3;

}

// Finds the intersection of two lines, or returns false.
// The line is defined in general form at cv::Point3f(a,b,c) with 'a y + b x + c = 0'
// and the segment by p1 and p2.
bool HullToObjectModule::lineSegmentIntersection(cv::Point3f &line,
                             cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &r) {

    // if b == 0 Horizontal line
    if(line.y == 0) {
        if(line.x == 0)
            return false;
        float y = -line.z/line.x;
        if(p1.y == p2.y) { //Parallel?
            if(fabs(p1.y - y) < /*EPS*/1e-8) {
                r.x = (p1.x + p2.x)/2.0;
                r.y = p1.y;
                return true;
            } else //Parallel
                return false;
        }
        //Not parallel
        //Check if intercept is in interval
        if(p1.y > p2.y) {
            if(p2.y > y || y > p1.y)
                return false;
        } else {
            if(p1.y > y || y > p2.y)
                return false;
        }
        if(p2.x - p1.x == 0) {
            r.x = p1.x;
            r.y = y;
        } else {
            float m = (p2.y - p1.y)/(p2.x - p1.x), n = p1.y - m*p1.x;
            r.x = (y - n)/m;
            r.y = y;
            return true;
        }
    } else if(line.x == 0) { //a == 0 Vertical line
        float x = -line.z/line.y;
        if(p1.x == p2.x) { //Parallel?
            if(fabs(p1.x - x) < /*EPS*/1e-8) {
                r.x = p1.x;
                r.y = (p1.y + p2.y)/2.0;
                return true;
            } else //Parallel
                return false;
        }
        //Not parallel
        //Check if intercept is in interval
        if(p1.x > p2.x) {
            if(p2.x > x || x > p1.x)
                return false;
        } else {
            if(p1.x > x || x > p2.x)
                return false;
        }
        float m = (p2.y - p1.y)/(p2.x - p1.x), n = p1.y - m*p1.x;
        r.x = x;
        r.y = m*x + n;
        return true;
    }
    //General case

    float m1 = -line.y/line.x, n1 = -line.z/line.x;

    if(p2.x - p1.x == 0) {
        r.x = p1.x;
        r.y = r.x*m1 + n1;
    } else {
        /*if(fabs(p2.y - p1.y) == 0.0) {
            r.y = p1.y;
            r.x = (r.y - n1)/m1;
            return true;
        }*/
        float m2 = (p2.y - p1.y)/(p2.x - p1.x),
              n2 = p1.y - m2*p1.x;

        if (fabs(m1 - m2) < /*EPS*/1e-8)
            return false;

        r.x = (n1 - n2)/(m2 - m1);
        r.y = m1*r.x + n1;
    }

    //Check if point is in interval
    if(p1.y != p2.y) {
        if(p1.y > p2.y) {
            if(p2.y > r.y || r.y > p1.y)
                return false;
        } else {
            if(p1.y > r.y || r.y > p2.y)
                return false;
        }
    }

    if(p1.x > p2.x) {
        if(p2.x > r.x || r.x > p1.x)
            return false;
    } else {
        if(p1.x > r.x || r.x > p2.x)
            return false;
    }

    return true;
}

//a y + b x + c = 0
bool HullToObjectModule::getGeneralLineForm(cv::Point2f p1, cv::Point2f p2,
                                                    cv::Point3f &rline) {

    // Horizontal line
    if(fabs(p1.y - p2.y) < /*EPS*/1e-8) {
        if(fabs(p1.x - p2.x) < /*EPS*/1e-8)
            return false;
        rline.x = 1.0;   //a
        rline.y = 0.0;   //b
        rline.z = -p1.y; //c
        return true;
    } else if(fabs(p1.x - p2.x) < /*EPS*/1e-8) { //b == 0 Vertical line
        rline.x = 0.0;   //a
        rline.y = 1.0;   //b
        rline.z = -p1.x; //c
        return true;
    }

    rline.x = p2.x - p1.x;
    rline.y = p1.y - p2.y;
    rline.z = p2.y*p1.x - p1.y*p2.x;
    return true;
}


bool HullToObjectModule::getGeneralLineForm(float x, float y,
                                                    float dx, float dy,
                                                    cv::Point3f &rline) {

    // Horizontal line
    if(fabs(dy) < /*EPS*/1e-8) {
        if(fabs(dx) < /*EPS*/1e-8)
            return false;
        rline.x = 1.0;   //a
        rline.y = 0.0;   //b
        rline.z = -y; //c
    } else if(dx < /*EPS*/1e-8) { //b == 0 Vertical line
        rline.x = 0.0;   //a
        rline.y = 1.0;   //b
        rline.z = -x; //c
    }

    rline.x = dx;
    rline.y = -dy;
    rline.z = dy*x - dx*y;
    return true;
}


double HullToObjectModule::distanceToSegment(int i1, int i2, std::vector<cv::Point> &contour) {
    int i, x, y, dx, dy, x1, y1, x2, y2, n = contour.size();
    double sum = 0;

    x1 = contour[i1%n].x;
    y1 = contour[i1%n].y;
    x2 = contour[i2%n].x;
    y2 = contour[i2%n].y;
    dx = x2 - x1;
    dy = y2 - y1;
    if(dx == 0) {
        for(i=i1+1; i<i2; i++)
            sum += fabs(contour[i].x - x1);
    } else if(dy == 0) {
        for(i=i1+1; i<i2; i++)
            sum += fabs(contour[i].y - y1);
    } else {
        double m = dy/(double)dx, n = y1 - m*x1, f = sqrt(m*m + 1);
        for(i=i1+1; i<i2; i++)
            sum += fabs(m*contour[i].x + n - contour[i].y)/f;
    }

    return sum;
}



bool HullToObjectModule::movementFound(cv::Mat f, int wsize, int i0, int j0) {
    int i, j, diff = wsize/2;
    int w = f.cols, h = f.rows;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<h)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < w)
                    if(f.data[i*f.step + j] != 0)
                        return true;
    return false;
}

bool HullToObjectModule::movementFound(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
    int i, j, diff = wsize/2,
        y1 = f.rows, x1 = f.cols;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<y1)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < x1)
                    if(f.data[i*f.step + j] != 0)
                        return true;
    return false;
}

int HullToObjectModule::movementCount(cv::Mat f, int wsize, int i0, int j0, cv::Rect roi) {
    int i, j, diff = wsize/2, count = 0,
        y1 = f.rows, x1 = f.cols;

    for(i=i0-diff; i<=i0+diff; i++)
        if(i>=0 && i<y1)
            for(j=j0-diff; j<=j0+diff; j++)
                if(j>=0  && j < x1)
                    if(f.data[i*f.step + j] != 0)
                        count++;
    return count;
}


double HullToObjectModule::histogramDistance(cv::MatND h1, cv::MatND h2) {
    cv::MatND d(h1.size(), CV_32FC1);
    cv::absdiff(h1, h2, d);
    cv::Scalar s = cv::sum(d);
    return s.val[0];
}

