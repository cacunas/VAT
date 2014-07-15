#ifndef _GEOMETRIC_H_
#define _GEOMETRIC_H_

#include <vector>
#include <cmath>
#include <cfloat>
#include <cstring>
#include <iostream>
#include "common.h"
#include "calibration.h"

//#define M_PI    3.14159265358979323846 // why would you redefine M_PI? WTF!
//Intersection error defined as sqrt(2)/2, as used in function pointxy_in_segment_strict
#define MAX_ERROR_TO_INTERSECTION 0.707106781186547524381894
#define MAX_FLOAT_ERROR 0.01
#define MAX_FLOAT_ERROR_LITTLE_NUMBERS 0.000001

template <typename T>
class Rectangle;

template <typename T>
class line2D {
    public:
        line2D();
        line2D(T i_x1, T i_y1, T i_x2, T i_y2);
        ~line2D();

        T x1;
        T y1;
        T x2;
        T y2;
};

template <typename T>
class Intersection {
    public:
        Intersection();
        Intersection(T i_x1, T i_y1, T i_x2, T i_y2);
        ~Intersection();

        T x1, y1;
        T x2, y2;
        T distance;
        int type; // 1: One point intersection (Stored in (x1,y1) )
                  // 0: Segment intersection (Limits stored in {(x1,y1);(x2,y2)} )
};

template <typename T>
class point2D {
    public:
        point2D();
        point2D(T i_x, T i_y);
        ~point2D();
        point2D& operator=(const point2D& p);
        static double distance(point2D *p1, point2D *p2);
        static bool isCCWPts(point2D *pPt0, point2D *pPt1, point2D *pPt2);
        static bool isCCW(double x0, double y0, double x1, double y1, double x2, double y2);
        static bool processIntersectionLines(double x1, double y1, double x2, double y2,
                                             double x3, double y3, double x4, double y4,
                                             point2D *pIntersection);
        static bool isLinesIntersection(double x1, double y1, double x2, double y2,
                                        double x3, double y3, double x4, double y4,
                                        point2D *pIntersection);
        static bool pointInSegment(double x, double y, point2D *begin, point2D *end);
        double pointLineDistance(point2D *lpt1, point2D *lpt2);
        static bool pointInSegmentStrict(T x, T y, point2D *begin, point2D *end);
        double pointSegmentDistance(point2D *begin, point2D *end);
        static Intersection<T> *getSegmentsIntersection(point2D *p1L1, point2D *p2L1, point2D *p1L2, point2D *p2L2);
        static Intersection<T> *getPreciseSegmentsIntersection(point2D *p1L1, point2D *p2L1, point2D *p1L2, point2D *p2L2);
        static bool pointPreciselyInSegmentStrict(double x, double y, point2D *begin, point2D *end);

        T x;
        T y;
};

template <typename T>
class point3D {
    public:
        point3D();
        point3D(T i_x, T i_y, T i_z);
        ~point3D();
        point3D& operator=(const point3D& p);
        double distance(point3D *p1, point3D *p2);

        T x;
        T y;
        T z;
        bool equalPoints3D(point3D *p1, point3D *p2);
};

/*class h_segment {
    public:
        h_segment();
        h_segment(int i_line, int i_left, int i_right);
        ~h_segment();

        int line;
        int left;
        int right;
};*/


template <typename T>
class segment2D {
    public:
        segment2D();
        segment2D(point2D<T>& i_first, point2D<T>& i_last);
        ~segment2D();

        point2D<T> first;
        point2D<T> last;
};

template <typename T>
class segment3D {
    public:
        segment3D();
        segment3D(point3D<T>& i_first, point3D<T>& i_last);
        ~segment3D();

        point3D<T> first;
        point3D<T> last;
};


template <typename T>
class Rectangle {
  public:
    Rectangle();
    Rectangle(T x, T y, T w, T h);
    ~Rectangle();
    void initRectangle(T x, T y, T w, T h);
    Rectangle *copyRectangle();

    Rectangle *mergeRectangle(Rectangle *rect);
    static Rectangle *mergeRectangles(Rectangle *r1, Rectangle *r2);

    void absorbRectangle(Rectangle *r);
    void absorbRectangle(Rectangle &r);

    static double rectangleDistance(Rectangle *r1, Rectangle *r2);
    double rectangleDistance(Rectangle *r);

    static double rectangleGravityDistance(Rectangle *r1, Rectangle *r2);
    double rectangleGravityDistance(Rectangle *r);

    static T horizontalDistance(Rectangle *r1, Rectangle *r2);
    static T verticalDistance(Rectangle *r1, Rectangle *r2);


    static Rectangle *rectanglesIntersection(Rectangle *r1, Rectangle *r2);
    static bool rectanglesIntersection(Rectangle *r1, Rectangle *r2, Rectangle *inter);
    Rectangle *rectangleIntersection(Rectangle *r);

    static Rectangle *rectanglesUnion(Rectangle *r1, Rectangle *r2);
    Rectangle *rectangleUnion(Rectangle *r);

    static double rectangleIntersectRatio(Rectangle *r1, Rectangle *r2);
    double rectangleIntersectRatio(Rectangle *r);

    static double rectangleIntersectRatioStrict(Rectangle *r1, Rectangle *r2);
    double rectangleIntersectRatioStrict(Rectangle *r);

    static bool rectangleInRectangle(Rectangle *r1, Rectangle *r2);
    bool rectangleInRectangle(Rectangle *r);

    static double rectangleIntersectionArea(Rectangle *r1, Rectangle *r2);
    double rectangleIntersectionArea(Rectangle *r);

    static double rectangleNIntersectionArea(const std::vector<Rectangle *>& rectangles);

    double rectangleArea();

    int getPositionRelativeToCamera(SceneModel *smodel);

    int getPositionRelativeToCameraOld(SceneModel *smodel);

    void setEstimated3DPosition(SceneModel *smodel, point3D<double>& p, int position);

    void setPositionAtCenterBottom(SceneModel *smodel, point3D<double>& p);

    static bool isRect1InsideRect2(Rectangle *r1, Rectangle *r2);

    static bool thereIsIntersection(Rectangle *r1, Rectangle *r2);

    static T overlappingArea(Rectangle *r1, Rectangle *r2);

    bool pointInRectangle(T p_x, T p_y);


    T xleft;
    T ytop;
    T xright;
    T ybottom;
    T width;
    T height;
};

typedef Rectangle<int> rectangle;
typedef rectangle *rectangle_t;
typedef Rectangle<double> frectangle;
typedef frectangle *frectangle_t;
typedef point2D<double> Position2D;
typedef point3D<double> Position3D;



template <typename T>
class polygon2D {
    public:
        polygon2D();
        polygon2D(int reserve);
        polygon2D& operator=(polygon2D& p);
        polygon2D *copy();
        ~polygon2D();
        void computeBoundingRectangle(Rectangle<T>& b);
        void computeBoundingRectangle();
        static polygon2D *intersectionPolygon(polygon2D *p1, polygon2D *p2);
        bool getIntersectionsWithConvexPolygon(double val, bool vertical, double& v1, double& v2);
        bool counterclockwisePolygon();
        polygon2D *getInversedPolygon();
        bool pointInPolygon(T x, T y, bool strict);
        static bool pointPreciselyInConvexPolygon(double x, double y, polygon2D *poly, bool strict);
        static bool equalPolygons(polygon2D *p1, polygon2D *p2, double allowed_point_error);
        double polygonArea();
        bool pointInConvexPolygon(T x, T y, bool strict);

        static polygon2D *rectanglesIntersection(polygon2D *rectangle1, polygon2D *rectangle2);
        bool memmove(int to, int from, int number);
        QSharedPointer<polygon2D<int> > projectImagePolygon2D(homography_matrix H);

        bool bb_done;

        Rectangle<T> boundingBox;
        std::vector< point2D<T> > points;
};

typedef Rectangle<int> BoundingBox;

template <typename T>
class polygon3D {
  public:
    polygon3D();
    polygon3D(int reserve);
    polygon3D& operator=(polygon3D& p);
    ~polygon3D();

    QSharedPointer<polygon2D<T> > makePolygon2D();
    QSharedPointer<polygon2D<int> > projectPolygon2D(perspective_matrix M);

    Rectangle<T> boundingBox;
    std::vector< point3D<T> > points;
};

// for points 2D
#define POINT_2D_X(p)               ((p)->x)
#define POINT_2D_Y(p)               ((p)->y)

// for points 3D
#define POINT_3D_X(p)               ((p)->x)
#define POINT_3D_Y(p)               ((p)->y)
#define POINT_3D_Z(p)               ((p)->z)


// for 2d and 3d polygon...
#define POLYGON_N_POINTS(pl)		((pl)->points.size())
#define POLYGON_NTH_POINT(pl, n)	(&(pl)->points[n])

// only for 2d-polygon 
#define POLYGON_2D_BB_DONE(pl)		    ((pl)->bb_done)
#define POLYGON_2D_BB(pl)		    ((pl)->boundingBox)
#define POLYGON_2D_BB_X(pl)		    RECT_X(&((pl)->boundingBox))
#define POLYGON_2D_BB_Y(pl)		    RECT_Y(&((pl)->boundingBox))
#define POLYGON_2D_BB_WIDTH(pl)		RECT_WIDTH(&((pl)->boundingBox))
#define POLYGON_2D_BB_HEIGHT(pl)	RECT_HEIGHT(&((pl)->boundingBox))
#define POLYGON_2D_BB_XLEFT(pl)		RECT_XLEFT(&((pl)->boundingBox))
#define POLYGON_2D_BB_XRIGHT(pl)	RECT_XRIGHT(&((pl)->boundingBox))
#define POLYGON_2D_BB_YBOTTOM(pl)	RECT_YBOTTOM(&((pl)->boundingBox))
#define POLYGON_2D_BB_YTOP(pl)          RECT_YTOP(&((pl)->boundingBox))
#define POLYGON_2D_BB_XCENTER(pl)	(POLYGON_2D_BB_XLEFT(pl) + POLYGON_2D_BB_WIDTH(pl) / 2.0)
#define POLYGON_2D_BB_YCENTER(pl)   (POLYGON_2D_BB_YTOP(pl) + POLYGON_2D_BB_HEIGHT(pl) / 2.0)	
#define RECT_XLEFT(_rect)	((_rect)->xleft)
#define RECT_X(_rect)		RECT_XLEFT(_rect)
#define RECT_XRIGHT(_rect)	((_rect)->xright)
#define RECT_YTOP(_rect)	((_rect)->ytop)
#define RECT_Y(_rect)		RECT_YTOP(_rect)
#define RECT_YBOTTOM(_rect)	((_rect)->ybottom)
#define RECT_WIDTH(_rect)	((_rect)->width)
#define RECT_HEIGHT(_rect)	((_rect)->height)
#define RECT_XCENTER(_rect)	(RECT_XLEFT(_rect) + RECT_WIDTH(_rect)/2)
#define RECT_YCENTER(_rect)	(RECT_YTOP(_rect) + RECT_HEIGHT(_rect)/2)

//IMPLEMENTATIONS

template <typename T>
line2D<T>::line2D() : x1(0), y1(0), x2(0), y2(0) {}

template <typename T>
line2D<T>::line2D(T i_x1, T i_y1, T i_x2, T i_y2) : x1(i_x1), y1(i_y1),
                                         x2(i_x2), y2(i_y2) {}
template <typename T>
line2D<T>::~line2D() {}

template <typename T>
point2D<T>::point2D() : x(0), y(0) {}

template <typename T>
point2D<T>::point2D(T i_x, T i_y) : x(i_x), y(i_y) {}

template <typename T>
point2D<T>& point2D<T>::operator=(const point2D<T>& p) {
    x = p.x;
    y = p.y;
    return *this;
}

template <typename T>
double point2D<T>::distance(point2D<T> *p1, point2D<T> *p2) {
    double
        dx = POINT_2D_X(p1) - POINT_2D_X(p2),
        dy = POINT_2D_Y(p1) - POINT_2D_Y(p2);
    return sqrt(dx*dx + dy*dy);
}

template <typename T>
point2D<T>::~point2D() {}

template <typename T>
point3D<T>::point3D() : x(0), y(0), z(0) {}

template <typename T>
point3D<T>::point3D(T i_x, T i_y, T i_z) : x(i_x), y(i_y), z(i_z) {}

template <typename T>
point3D<T>& point3D<T>::operator=(const point3D<T>& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    return *this;
}

template <typename T>
double point3D<T>::distance(point3D<T> *p1, point3D<T> *p2) {
    double
        dx = POINT_3D_X(p1) - POINT_3D_X(p2),
        dy = POINT_3D_Y(p1) - POINT_3D_Y(p2),
        dz = POINT_3D_Z(p1) - POINT_3D_Z(p2);
    return sqrt(dx*dx + dy*dy + dz*dz);
}

template <typename T>
point3D<T>::~point3D() {}
/*
h_segment::h_segment() : line(0), left(0), right(0) {}

h_segment::h_segment(int i_line, int i_left, int i_right)  : line(i_line),
                                                             left(i_left),
                                                             right(i_right) {}

h_segment::~h_segment() {}
*/
template <typename T>
Intersection<T>::Intersection(): x1(0), y1(0), x2(0), y2(0),
                                 distance(0), type(1) {}

template <typename T>
Intersection<T>::Intersection(T i_x1, T i_y1,
                              T i_x2, T i_y2): x1(i_x1), y1(i_y1),
                                               x2(i_x2), y2(i_y2) {
    T dx = x1 - x2, dy = y1 - y2;
    distance = sqrt(dx + dy);
}

template <typename T>
Intersection<T>::~Intersection() {}

template <typename T>
segment2D<T>::segment2D() {}

template <typename T>
segment2D<T>::segment2D(point2D<T>& i_first, point2D<T>& i_last) : first(i_first), last(i_last) {}

template <typename T>
segment2D<T>::~segment2D() {}

template <typename T>
segment3D<T>::segment3D() {}

template <typename T>
segment3D<T>::segment3D(point3D<T>& i_first, point3D<T>& i_last) : first(i_first), last(i_last) {}

template <typename T>
segment3D<T>::~segment3D() {}


template <typename T>
polygon2D<T>::polygon2D() : bb_done(false) {}

template <typename T>
polygon2D<T>& polygon2D<T>::operator=(polygon2D<T>& p) {
    memcpy(this, &p, sizeof(polygon2D<T>));
    points = p.points;
    return *this;
}

template <typename T>
polygon2D<T> *polygon2D<T>::copy() {
    polygon2D<T> *p = new polygon2D<T>(points.size());
    memcpy(p, this, sizeof(polygon2D<T>));
    p->points = points;
    return p;
}

template <typename T>
polygon2D<T>::polygon2D(int reserve): bb_done(false) {
    points.resize(reserve);
}


template <typename T>
polygon2D<T>::~polygon2D() {}

//Compute the bounding rectangle of a polygon
template <typename T>
void polygon2D<T>::computeBoundingRectangle(Rectangle<T>& b) {
    if (bb_done)
        memcpy(&b, &boundingBox, sizeof(Rectangle<int>));
    else {
        int n = points.size();

        if(n == 0) {
            memset(&b, 0, sizeof(Rectangle<int>));
            memset(&boundingBox, 0, sizeof(Rectangle<int>));
            bb_done = false;
            return;
        }

        double xmin, xmax, ymin, ymax;
        point2D<T> *current;
        typename std::vector< point2D<T> >::iterator it, it_end = points.end();

        xmin = DBL_MAX / 2.0;
        xmax = - xmin;
        ymin =   xmin;
        ymax =   xmax;

        for (it = points.begin(); it != it_end; it++) {
            current = &(*it);
            xmin = POINT_2D_X(current) < xmin ? POINT_2D_X(current) : xmin;
            xmax = POINT_2D_X(current) > xmax ? POINT_2D_X(current) : xmax;
            ymin = POINT_2D_Y(current) < ymin ? POINT_2D_Y(current) : ymin;
            ymax = POINT_2D_Y(current) > ymax ? POINT_2D_Y(current) : ymax;
        }

        RECT_X(&b) = RECT_X(&boundingBox) = xmin;
        RECT_Y(&b) = RECT_Y(&boundingBox) = ymin;
        RECT_WIDTH(&b) = RECT_WIDTH(&boundingBox) = xmax - xmin + 1;
        RECT_HEIGHT(&b) = RECT_HEIGHT(&boundingBox) = ymax - ymin + 1;
        RECT_XRIGHT(&b) = RECT_XRIGHT(&boundingBox) = xmax;
        RECT_YBOTTOM(&b) = RECT_YBOTTOM(&boundingBox) = ymax;
        bb_done = true;
    }
}

//Compute the bounding rectangle of a polygon (internal data)
template <typename T>
void polygon2D<T>::computeBoundingRectangle() {

    int n = points.size();

    if(n == 0) {
        memset(&boundingBox, 0, sizeof(Rectangle<int>));
        bb_done = false;
        return;
    }

    double xmin, xmax, ymin, ymax;
    point2D<T> *current;
    typename std::vector< point2D<T> >::iterator it, it_end = points.end();

    xmin = DBL_MAX / 2.0;
    xmax = - xmin;
    ymin =   xmin;
    ymax =   xmax;

    for (it = points.begin(); it != it_end; it++) {
        current = &(*it);
        xmin = POINT_2D_X(current) < xmin ? POINT_2D_X(current) : xmin;
        xmax = POINT_2D_X(current) > xmax ? POINT_2D_X(current) : xmax;
        ymin = POINT_2D_Y(current) < ymin ? POINT_2D_Y(current) : ymin;
        ymax = POINT_2D_Y(current) > ymax ? POINT_2D_Y(current) : ymax;
    }

    RECT_X(&boundingBox) = xmin;
    RECT_Y(&boundingBox) = ymin;
    RECT_WIDTH(&boundingBox) = xmax - xmin + 1;
    RECT_HEIGHT(&boundingBox) = ymax - ymin + 1;
    RECT_XRIGHT(&boundingBox) = xmax;
    RECT_YBOTTOM(&boundingBox) = ymax;
    bb_done = true;
}

// Optimized functions for convex polygon (requires ordered points of a convex polygon)
template <typename T>
double polygon2D<T>::polygonArea() {
    int i, j, n = points.size();
    double area = 0.0;

    for (i = 0; i < n; i++) {
        j = (i ==  n - 1 ? 0 : i + 1);
        area += points[i].x * points[j].y - points[i].y * points[j].x;
    }

    return fabs(area) / 2.0;
}

//Checks if a point (x,y) is inside a polygon
template <typename T>
bool polygon2D<T>::pointInConvexPolygon(T x, T y, bool strict) {

    int i, pos = 0, neg = 0;
    int num_points = points.size();
    double xp1, xp2, yp1, yp2;
    point2D<T> *p1, *p2;

    if(!this->boundingBox.pointInRectangle(x, y))
        return 0;

    p2 = &points[0];

    for(i=0;i<num_points;i++) {
        p1 = p2;
        p2 = &points[num_points - 1 ? i + 1 : 0];

        if (point2D<T>::pointInSegmentStrict(x, y, p1, p2))
            return (strict ? 0 : 1);

        xp1 = POINT_2D_X(p1);    yp1 = POINT_2D_Y(p1);
        xp2 = POINT_2D_X(p2);    yp2 = POINT_2D_Y(p2);

        if (((yp1<=y) && (y<yp2)) || ((yp2<=y) && (y<yp1))) {
            if ( x*(yp2 - yp1) + xp2*yp1 - xp1*yp2 < y*(xp2 - xp1) )
                pos++;
            else
                neg++;
        }

        if(neg && pos)
            return false;
    }

    return true;
}

//Checks a point in a polygon
template <typename T>
bool polygon2D<T>::pointPreciselyInConvexPolygon(double x, double y, polygon2D<T> *poly, bool strict) {
    int i, pos = 0, neg = 0;
    int num_points = POLYGON_N_POINTS(poly);
    double xp1, xp2, yp1, yp2;
    point2D<T> *p1, *p2;

    if(!POLYGON_2D_BB(poly).pointInRectangle(x, y))
        return false;

    p2 = POLYGON_NTH_POINT(poly, 0);

    for(i=0;i<num_points;i++) {
        p1 = p2;
        p2 = POLYGON_NTH_POINT(poly, i<num_points - 1 ? i + 1 : 0);

        if (point2D<T>::pointPreciselyInSegmentStrict(x, y, p1, p2))
            return (strict ? 0 : 1);

        xp1 = POINT_2D_X(p1);    yp1 = POINT_2D_Y(p1);
        xp2 = POINT_2D_X(p2);    yp2 = POINT_2D_Y(p2);

        if ( (yp1 <= y && y < yp2) || (yp2 <= y && y < yp1) ) {
            if ( x*(yp2 - yp1) + xp2*yp1 - xp1*yp2 < y*(xp2 - xp1) )
                pos++;
            else
                neg++;
        }

        if(neg && pos)
            return false;
    }

    return true;
}


// Get the intersection points with a polygon
template <typename T>
bool polygon2D<T>::getIntersectionsWithConvexPolygon(double val, bool vertical, double& v1, double& v2) {

    //First check if it is in the zone of the roof
    if(vertical) {
        if( val < RECT_XLEFT(&boundingBox) || val > RECT_XRIGHT(&boundingBox))
            return false;
    } else {
        if( val < RECT_YTOP(&boundingBox) || val > RECT_YBOTTOM(&boundingBox))
            return false;
    }

    int i, obtained_points = 0;
    int num_points = points.size();
    double xp1, xp2, yp1, yp2;
    point2D<T> *p1, *p2;
    double min, max;

    p2 = &points[0];

    for(i=0;i<num_points;i++) {
        p1 = p2;
        p2 = &points[i < num_points - 1 ? i + 1 : 0];

        xp1 = POINT_2D_X(p1);    yp1 = POINT_2D_Y(p1);
        xp2 = POINT_2D_X(p2);    yp2 = POINT_2D_Y(p2);

        if(vertical) {
            if(xp2 > xp1) {
                min = xp1; max = xp2;
            } else {
                min = xp2; max = xp1;
            }
            if(xp2 - xp1 == 0.0) //Ignore if segment is vertical (there must be two other intersecting it)
                continue;
            else {
                if ((val >=  min) && (val <= max)) {
                    if(!obtained_points) {
                        v1 = (val*(yp2 - yp1) + xp2*yp1 - xp1*yp2)/(xp2 - xp1);
                        obtained_points++;
                    } else {
                        v2 = (val*(yp2 - yp1) + xp2*yp1 - xp1*yp2)/(xp2 - xp1);
                        obtained_points++;
                    }
                }
            }
        } else { //Horizontal
            if(yp2 > yp1) {
                min = yp1; max = yp2;
            } else {
                min = yp2; max = yp1;
            }
            if(yp2 - yp1 == 0.0) //Ignore if segment is horizontal (there must be two other intersecting it)
                continue;
            else {
                if ((val >=  min) && (val <= max)) {
                    if(!obtained_points) {
                        v1 = ( val*(xp2 - xp1) - xp2*yp1 + xp1*yp2 ) / (yp2 - yp1);
                        obtained_points++;
                    } else {
                        v2 = ( val*(xp2 - xp1) - xp2*yp1 + xp1*yp2 ) / (yp2 - yp1);
                        obtained_points++;
                    }
                }
            }
        }

        if(obtained_points == 2)
            return true;
    }

    return false;
}


//This function allows a maximum error of allowed_point_error of distance
//error between equal points. This is done for robustness with respect to floating point errors in despite
//of a little loss of precision.
template <typename T>
bool polygon2D<T>::equalPolygons(polygon2D<T> *p1, polygon2D<T> *p2, double allowed_point_error) {
    int i, j;
    int found = 0;
    int n_points = p1->points.size(), n_points2 = p2->points.size();

    if(n_points != n_points2)
        return false;

    for(i=0; i<n_points; i++)
        if(    fabs(p1->points[0].x - p2->points[i].x) < allowed_point_error
            && fabs(p1->points[0].y - p2->points[i].y) < allowed_point_error) {
            found = 1;
            j = i;
            break;
        }
    if(found == 0)
        return false;

    for(i=1,j=(j+1)%n_points; i<n_points; i++, j=(j+1)%n_points)
        if(    fabs(p1->points[i].x - p2->points[j].x) >= allowed_point_error
            || fabs(p1->points[i].y - p2->points[j].y) >= allowed_point_error)
            return false;

    return true;
}

//Function which returns a polygon representing the area of intersection of two convex polygons (with any orientation).
//Both polygons must be clockwise or counter-clockwise at the same time.
//TO-DO!!!! pass doubles to int, specifying the precision required. e.g. 150.4562, 132.5,
// with precision 2, works with numbers, 15045.62, 13250, and will transform
//to double by dividing by 100.0 all numbers :D
template <typename T>
polygon2D<T> *polygon2D<T>::intersectionPolygon(polygon2D<T> *p1, polygon2D<T> *p2) {

    //First check polygons bounding boxes, if there is no bounding boxes intersection, there is no polygon intersection
    if(!Rectangle<T>::thereIsIntersection(&POLYGON_2D_BB(p1), &POLYGON_2D_BB(p2)))
        return NULL;

    if(polygon2D<T>::equalPolygons(p1, p2, 0.001))
        return p1->copy();

   int i;
   int num_inner_pol1 = 0, num_inner_pol2 = 0;
   int p1_points = p1->points.size(), p2_points = p2->points.size();
   int inner_points_p1[p1_points], inner_points_p2[p2_points];
   point2D<T> *p1pol1, *p1pol2;

   memset(inner_points_p1, 0, p1_points*sizeof(int));
   memset(inner_points_p2, 0, p2_points*sizeof(int));

   //Set inner point flags for both rectangles. An inner point can not be an intersecting point
   for(i=0; i<p1_points; i++) {
        p1pol1 = POLYGON_NTH_POINT(p1, i);
        if(polygon2D<T>::pointPreciselyInConvexPolygon(p1pol1->x, p1pol1->y, p2, 1)) {
            num_inner_pol1++;
            inner_points_p1[i] = 1;
        }
   }

   for(i=0; i<p2_points; i++) {
        p1pol2 = POLYGON_NTH_POINT(p2, i);
        if(polygon2D<T>::pointPreciselyInConvexPolygon(p1pol2->x, p1pol2->y, p1, 1)) {
            num_inner_pol2++;
            inner_points_p2[i] = 1;
        }
   }

   if(num_inner_pol1 == p1_points)
        return p1->copy();

   if(num_inner_pol2 == p2_points)
        return p2->copy();

   int max_points = 2*(p1_points + p2_points);
   polygon2D<T> *intersection_polygon = new polygon2D(max_points);
   int j, numIntersections = 0, num_current_intersections, num_allocated = 0, add;
   Intersection<T> *icurrent, *allocatedResults[max_points];
   double xnear, ynear, xfar, yfar;
   double dx, dy, d1, d2;
   point2D<T> *p2pol1, *p2pol2;

   //If no inner points for both rectangles, search for intersections
   if( num_inner_pol1 == 0 && num_inner_pol2 == 0 ) {

        point2D<T> current_intersections[2];

        p2pol1 = POLYGON_NTH_POINT(p1, 0);

        for(i=0; i<p1_points; i++) {
            p1pol1 = p2pol1;
            p2pol1 = POLYGON_NTH_POINT(p1, (i + 1)%p1_points);

            num_current_intersections = 0;

            p2pol2 = POLYGON_NTH_POINT(p2, 0);
            for(j=0; j<p2_points; j++) {
                p1pol2 = p2pol2;
                p2pol2 = POLYGON_NTH_POINT(p2, (j + 1)%p2_points);

                //If no intersection found, pass to the next pair
                if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol1, p2pol1, p1pol2, p2pol2)) == NULL )
                    continue;

                allocatedResults[num_allocated++] = icurrent;

                if(icurrent->type == 1) {
                    current_intersections[num_current_intersections  ].x = icurrent->x1;
                    current_intersections[num_current_intersections++].y = icurrent->y1;
                } else { //If there is line intersection, we have all the possible intersection points for two segments
                    num_current_intersections = 0;
                    current_intersections[num_current_intersections  ].x = icurrent->x1;
                    current_intersections[num_current_intersections++].y = icurrent->y1;
                    current_intersections[num_current_intersections  ].x = icurrent->x2;
                    current_intersections[num_current_intersections++].y = icurrent->y2;
                    break;
                }

                if(num_current_intersections > 2)
                    std::cout << "ERROR!!!: Exceded intersections capacity." << std::endl;
            }

            if(num_current_intersections == 0)
                continue;

            if(    num_current_intersections == 2
                && current_intersections[0].x == current_intersections[1].x
                && current_intersections[0].y == current_intersections[1].y )
                num_current_intersections = 1;

            if(num_current_intersections == 1) {
                xnear = current_intersections[0].x;
                ynear = current_intersections[0].y;

                add = 1;

                if(   numIntersections > 0  //Check if previous intersection is the same point
                   && intersection_polygon->points[numIntersections - 1].x == xnear
                   && intersection_polygon->points[numIntersections - 1].y == ynear )
                    add = 0;
                    //In case that the point is the last intersection, check against the first point
                else if(    numIntersections > 1
                         && intersection_polygon->points[0].x == xnear
                         && intersection_polygon->points[0].y == ynear )
                    add = 0;

                if(add) {
                    intersection_polygon->points[numIntersections].x = xnear;
                    intersection_polygon->points[numIntersections++].y = ynear;
                }
            } else {
                dx = current_intersections[0].x - p1pol1->x;
                dy = current_intersections[0].y - p1pol1->y;
                d1 = dx*dx + dy*dy;
                dx = current_intersections[1].x - p1pol1->x;
                dy = current_intersections[1].y - p1pol1->y;
                d2 = dx*dx + dy*dy;
                if(d1 < d2) {
                    xnear = current_intersections[0].x;
                    ynear = current_intersections[0].y;
                    xfar  = current_intersections[1].x;
                    yfar  = current_intersections[1].y;
                } else {
                    xnear = current_intersections[1].x;
                    ynear = current_intersections[1].y;
                    xfar  = current_intersections[0].x;
                    yfar  = current_intersections[0].y;
                }

                if(    numIntersections > 0  //Check if previous intersection is the same point
                    && (    intersection_polygon->points[numIntersections - 1].x != xnear
                         || intersection_polygon->points[numIntersections - 1].y != ynear ) ) {
                    intersection_polygon->points[numIntersections].x = xnear;
                    intersection_polygon->points[numIntersections++].y = ynear;
                } else if(numIntersections == 0) {
                    intersection_polygon->points[numIntersections].x = xnear;
                    intersection_polygon->points[numIntersections++].y = ynear;
                }

                //In case that the point is the last intersection, check against the first point
                if(    numIntersections > 1
                    && (    intersection_polygon->points[0].x != xfar
                         || intersection_polygon->points[0].y != yfar ) ) {
                    intersection_polygon->points[numIntersections].x = xfar;
                    intersection_polygon->points[numIntersections++].y = yfar;
                } else if(numIntersections <= 1) {
                    intersection_polygon->points[numIntersections].x = xfar;
                    intersection_polygon->points[numIntersections++].y = yfar;
                }
            }
        }

        for(i=0; i<num_allocated; i++)
            delete allocatedResults[i];

        if(numIntersections < 3) { //If the intersection can not form an area, the area is NULL
            delete intersection_polygon;
            return NULL;
        }

        return intersection_polygon;
    }


    /*   std::cout.precision(10);
    std::cout << "Inner points: \tp1(";
    for(i=0; i<p1_points; i++)
        std::cout << inner_points_p1[i] << " ";
    std::cout << ") \tp2(";
    for(i=0; i<p2_points; i++)
        std::cout << inner_points_p2[i] << " ";
    std::cout << ")" << std::endl;
    */

    int from_p1, inner_index;
    double p1_inner_min_distance[p1_points], p2_inner_min_distance[p2_points];
    double min, max = 0.0, dist;

    memset(p1_inner_min_distance, 0, p1_points*sizeof(double));
    memset(p2_inner_min_distance, 0, p2_points*sizeof(double));
    //Get the more distant inner point from the other rectangle segments, to avoid floating point precision problems

    //Check inner points of p2
    for(i=0; i<p2_points; i++) {
        if(inner_points_p2[i]) {
            min = DBL_MAX;
            p2pol1 = POLYGON_NTH_POINT(p1, 0);
            for(j=0; j<p1_points; j++) {
                p1pol1 = p2pol1;
                p2pol1 = POLYGON_NTH_POINT(p1, (j + 1)%p1_points);
                dist = POLYGON_NTH_POINT(p2, i)->pointSegmentDistance(p1pol1, p2pol1);
                if(dist < min)
                    min = dist;
            }
            p2_inner_min_distance[i] = min;
            if(min > max) {
                max = min;
                from_p1 = 0;
                inner_index = i;
            }
        }
    }

   //Check inner points of p1
   for(i=0; i<p1_points; i++) {
        if(inner_points_p1[i]) {
            min = DBL_MAX;
            p2pol2 = POLYGON_NTH_POINT(p2, 0);
            for(j=0; j<p2_points; j++) {
                p1pol2 = p2pol2;
                p2pol2 = POLYGON_NTH_POINT(p2, (j + 1)%p2_points);
                dist = POLYGON_NTH_POINT(p1, i)->pointSegmentDistance(p1pol2, p2pol2);
                if(dist < min)
                    min = dist;
            }
            p1_inner_min_distance[i] = min;
            if(min > max) {
                max = min;
                from_p1 = 1;
                inner_index = i;
            }
        }
    }

    point2D<T> *current_point;
    polygon2D<T> *p_inner, *p_other;
    int n_pinner, n_pother, i_inner, i_other, in_inner = 1, first_other = 1;
    int *inner_points_p_inner, *inner_points_p_other, differentLastToAdd = 1;
    int inner_in_limit, other_in_limit;
    //TO-DO!!! FIX WHEN icurrent == NULL
    //Example from frame 254969 of GERHOME beard man video.
    //BLOB 3, PERSON, CROUCHING, alpha = 0.1596865465, h = 41,25812530
    //inner_pol = base_pol, other_pol = object_pol,
    //Most inner point comes from rectangle 1 (point 3)
    //Good Sequence: inner_3 -> A -> other_0 -> B -> inner_2
    //B is not detected, the intersection between segs. other_{0,1} and inner_{1,2}
    //is not detected.
    //Recovery method must be implemented if there is no way to detect the intersection.

    if(from_p1) {
        p_inner = p1;
        p_other = p2;
        n_pinner = p1_points;
        n_pother = p2_points;
        current_point = POLYGON_NTH_POINT(p1, inner_index);
        inner_points_p_inner = inner_points_p1;
        inner_points_p_other = inner_points_p2;
    } else {
        p_inner = p2;
        p_other = p1;
        n_pinner = p2_points;
        n_pother = p1_points;
        current_point = POLYGON_NTH_POINT(p2, inner_index);
        inner_points_p_inner = inner_points_p2;
        inner_points_p_other = inner_points_p1;
    }

    //Insert inner point as the first point of the final solution
    intersection_polygon->points[numIntersections  ].x = current_point->x;
    intersection_polygon->points[numIntersections++].y = current_point->y;
    i_inner = inner_index;

    do {
        if(in_inner == 1) {
            p1pol1 = POLYGON_NTH_POINT(p_inner, i_inner);
            p2pol1 = POLYGON_NTH_POINT(p_inner, (i_inner + 1)%n_pinner);

            //If next point to check is inner point, add it immediately and pass to next segment
            if(inner_points_p_inner[(i_inner + 1)%n_pinner]) {
                i_inner = (i_inner + 1)%n_pinner;
                //If the next possible point to add is inner point and the same one added first,
                //the work is finished. :D
                if(i_inner == inner_index) {
                    differentLastToAdd = 0;
                    icurrent = NULL;
                } else { //Add the inner point because it is sure that it is part of the intersection solution
                    current_point = POLYGON_NTH_POINT(p_inner, i_inner);
                    intersection_polygon->points[numIntersections  ].x = current_point->x;
                    intersection_polygon->points[numIntersections++].y = current_point->y;
                    continue;
                }
            } else if(first_other) { //If next point is not inner point, check the intersections
                //Search for first intersection
                p2pol2 = POLYGON_NTH_POINT(p_other, 0);
                for(j=0; j<n_pother; j++) {
                    p1pol2 = p2pol2;
                    p2pol2 = POLYGON_NTH_POINT(p_other, (j + 1)%n_pother);
                    //If no intersection found, pass to the next segment
                    if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol1, p2pol1, p1pol2, p2pol2)) == NULL )
                        continue;
                    //Set the index for the first segment to analyse for the other rectangle
                    i_other = j;
                    first_other = 0;
                    allocatedResults[num_allocated++] = icurrent;
                    break;
                }
            } else {
                //Start intersection search from next point to not repeat the same intersection
                int aux_other;
                p2pol2 = POLYGON_NTH_POINT(p_other, i_other);
                for(j=0, aux_other = i_other; j < n_pother - 1; j++, aux_other = (aux_other + 1)%n_pother) {
                    p1pol2 = p2pol2;
                    p2pol2 = POLYGON_NTH_POINT(p_other, (aux_other + 1)%n_pother);
                    //If no intersection found, pass to the next segment
                    if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol1, p2pol1, p1pol2, p2pol2)) == NULL )
                        continue;
                    //Set the index for the first segment to analyse for the other rectangle
                    i_other = aux_other;
                    allocatedResults[num_allocated++] = icurrent;
                    break;
                }
            }

            if(icurrent == NULL) {  //An error???? It should not happen because if next point is out an intersection should exist.
                                    //If an error of this type occurs we stop the computation.
                //	 std::cout << "intersection_polygon: Error of intersection not found!!" << std::endl;
                break;
            } else if(icurrent->type == 1) { //A point intersection
                intersection_polygon->points[numIntersections  ].x = icurrent->x1;
                intersection_polygon->points[numIntersections++].y = icurrent->y1;
                //Decision for next segment and rectangle
                inner_in_limit = icurrent->x1 == p2pol1->x && icurrent->y1 == p2pol1->y ? 1 : 0;
                other_in_limit = 0;
                p1pol2 = POLYGON_NTH_POINT(p_other,  i_other       );
                p2pol2 = POLYGON_NTH_POINT(p_other, (i_other + 1)%n_pother);
                if(icurrent->x1 == p1pol2->x && icurrent->y1 == p1pol2->y)
                    other_in_limit = 1;
                else if(icurrent->x1 == p2pol2->x && icurrent->y1 == p2pol2->y) {
                    other_in_limit = 1;
                    i_other = (i_other + 1)%n_pother;
                }

                if(inner_in_limit) {
                    if(other_in_limit) //If intersection corresponds to a vertex in both rectangles, treat the special case
                        in_inner = -1;
                    else //If intersection point is not in limit of the other rectangle segment, no need to change of rectangle
                        in_inner = 1;
                } else  //If the intersection is not in limit, we have to change from inner to the other rectangle
                    in_inner = 0;
            } else { //If there is line intersection, we add the second point and treat the next point decision as a special case
                //Add the not added point, and based on this decide the next starting segment for both rectangles
                intersection_polygon->points[numIntersections  ].x = icurrent->x2;
                intersection_polygon->points[numIntersections++].y = icurrent->y2;

                i_other = (i_other + 1)%n_pother;

                p2pol2 = POLYGON_NTH_POINT(p_other, i_other);

                //If both ending points correspond to a vertex, treat special case
                if(p2pol1->x == p2pol2->x && p2pol1->y == p2pol2->y)
                    in_inner = -1;
                else {  //Nearest second to last added point will remain
                    double
                        dx_inner =  p2pol1->x - intersection_polygon->points[numIntersections - 1].x,
                        dx_other =  p2pol2->x - intersection_polygon->points[numIntersections - 1].x,
                        dy_inner =  p2pol1->y - intersection_polygon->points[numIntersections - 1].y,
                        dy_other =  p2pol2->y - intersection_polygon->points[numIntersections - 1].y;

                    if(dx_inner*dx_inner + dy_inner*dy_inner < dx_other*dx_other + dy_other*dy_other)
                        in_inner = 1;
                    else
                        in_inner = 0;
                }
            }

            i_inner = (i_inner + 1)%n_pinner;

        } else if (in_inner == 0) { //In the other rectangle

            p1pol2 = POLYGON_NTH_POINT(p_other, i_other);
            p2pol2 = POLYGON_NTH_POINT(p_other, (i_other + 1)%n_pother);

            //If next point to check is inner point, add it immediately and pass to next segment
            if(inner_points_p_other[(i_other + 1)%n_pother]) {
                i_other = (i_other + 1)%n_pother;
                current_point = POLYGON_NTH_POINT(p_other, i_other);
                intersection_polygon->points[numIntersections  ].x = current_point->x;
                intersection_polygon->points[numIntersections++].y = current_point->y;
                continue;
            } else {
                //Start intersection search from next point to not repeat the same intersection
                int aux_inner;
                p2pol1 = POLYGON_NTH_POINT(p_inner, i_inner);
                for(j=0, aux_inner = i_inner; j<(n_pinner - 1); j++, aux_inner = (aux_inner + 1)%n_pinner) {
                    p1pol1 = p2pol1;
                    p2pol1 = POLYGON_NTH_POINT(p_inner, (aux_inner + 1)%n_pinner);
                    //If no intersection found, pass to the next segment
                    if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol2, p2pol2, p1pol1, p2pol1)) == NULL )
                        continue;
                    //Set the index for the first segment to analyse for the other rectangle
                    i_inner = aux_inner;
                    allocatedResults[num_allocated++] = icurrent;
                    break;
                }
            }

            if(icurrent == NULL) { //An error???? It should not happen because if next point is out an intersection should exist.
                                   //If an error of this type occurs, finish computation and give the partial intersection found.
                std::cout << "Error of intersection not found!!" << std::endl;
                break;
            } else if(icurrent->type == 1) { //A point intersection
                intersection_polygon->points[numIntersections  ].x = icurrent->x1;
                intersection_polygon->points[numIntersections++].y = icurrent->y1;
                //Decision for next segment and rectangle
                other_in_limit = icurrent->x1 == p2pol2->x && icurrent->y1 == p2pol2->y ? 1 : 0;
                inner_in_limit = 0;
                p1pol1 = POLYGON_NTH_POINT(p_inner,  i_inner       );
                p2pol1 = POLYGON_NTH_POINT(p_inner, (i_inner + 1)%n_pinner);
                if(icurrent->x1 == p1pol1->x && icurrent->y1 == p1pol1->y)
                    inner_in_limit = 1;
                else if(icurrent->x1 == p2pol1->x && icurrent->y1 == p2pol1->y) {
                    inner_in_limit = 1;
                    i_inner = (i_inner + 1)%n_pinner;
                }

                if(other_in_limit) {
                    if(inner_in_limit) //If intersection corresponds to a vertex in both rectangles, treat the special case
                        in_inner = -1;
                    else //If intersection point is not in limit of the inner rectangle segment, no need to change of rectangle
                        in_inner = 0;
                } else  //If the intersection is not in limit, we have to change from other to the inner rectangle
                    in_inner = 1;
            } else { //If there is line intersection, we add the second point and treat the next point decision as a special case
                //Add the not added point, and based on this decide the next starting segment for both rectangles
                intersection_polygon->points[numIntersections  ].x = icurrent->x2;
                intersection_polygon->points[numIntersections++].y = icurrent->y2;

                i_inner = (i_inner + 1)%n_pinner;

                p2pol1 = POLYGON_NTH_POINT(p_inner, i_inner);

                //If both ending points correspond to a vertex, treat special case
                if(p2pol1->x == p2pol2->x && p2pol1->y == p2pol2->y)
                    in_inner = -1;
                else {  //Nearest second to last added point will remain
                    double
                        dx_inner =  p2pol1->x - intersection_polygon->points[numIntersections - 1].x,
                        dx_other =  p2pol2->x - intersection_polygon->points[numIntersections - 1].x,
                        dy_inner =  p2pol1->y - intersection_polygon->points[numIntersections - 1].y,
                        dy_other =  p2pol2->y - intersection_polygon->points[numIntersections - 1].y;

                    if(dx_inner*dx_inner + dy_inner*dy_inner < dx_other*dx_other + dy_other*dy_other)
                        in_inner = 1;
                    else
                        in_inner = 0;
                }
            }

            i_other = (i_other + 1)%n_pother;

        } else { //Last added point correspond to a vertex in both rectangles

            //If next point to check is inner point, add it immediately and pass to next segment
            if(inner_points_p_inner[(i_inner + 1)%n_pinner]) {
                i_inner = (i_inner + 1)%n_pinner;
                i_other = (i_other + 1)%n_pother;
                //If the next possible point to add is inner point and the same one added first,
                //the work is finished. :D
                if(i_inner == inner_index) {
                    differentLastToAdd = 0;
                    icurrent = NULL;
                } else { //Add the inner point because it is sure that it is part of the intersection solution
                    current_point = POLYGON_NTH_POINT(p_inner, i_inner);
                    intersection_polygon->points[numIntersections  ].x = current_point->x;
                    intersection_polygon->points[numIntersections++].y = current_point->y;
                    in_inner = 1;
                    continue;
                }
            } else if(inner_points_p_other[(i_other + 1)%n_pother]) {
                i_inner = (i_inner + 1)%n_pinner;
                i_other = (i_other + 1)%n_pother;
                current_point = POLYGON_NTH_POINT(p_other, i_other);
                intersection_polygon->points[numIntersections  ].x = current_point->x;
                intersection_polygon->points[numIntersections++].y = current_point->y;
                in_inner = 0;
                continue;
            } else {

                //Check if current segments perform a line intersection (A point intersection is warrantied because the first
                //point corresponds to a common vertex)

                //Start intersection search from next point to not repeat the same intersection
                p1pol1 = POLYGON_NTH_POINT(p_inner, i_inner);
                p2pol1 = POLYGON_NTH_POINT(p_inner, (i_inner + 1)%n_pinner);
                p1pol2 = POLYGON_NTH_POINT(p_other, i_other);
                p2pol2 = POLYGON_NTH_POINT(p_other, (i_other + 1)%n_pother);

                icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol2, p2pol2, p1pol1, p2pol1);

                if(icurrent != NULL && icurrent->type == 0) { //New line case to treat
                    //Add the not added point, and based on this decide the next starting segment for both rectangles
                    intersection_polygon->points[numIntersections  ].x = icurrent->x2;
                    intersection_polygon->points[numIntersections++].y = icurrent->y2;
                    i_inner = (i_inner + 1)%n_pinner;
                    i_other = (i_other + 1)%n_pother;
                    //If both ending points correspond to a vertex, treat special case
                    if(p2pol1->x == p2pol2->x && p2pol1->y == p2pol2->y)
                        in_inner = -1;
                    else {  //Nearest second to last added point will remain
                        double
                            dx_inner =  p2pol1->x - intersection_polygon->points[numIntersections - 1].x,
                            dx_other =  p2pol2->x - intersection_polygon->points[numIntersections - 1].x,
                            dy_inner =  p2pol1->y - intersection_polygon->points[numIntersections - 1].y,
                            dy_other =  p2pol2->y - intersection_polygon->points[numIntersections - 1].y;

                        if(dx_inner*dx_inner + dy_inner*dy_inner < dx_other*dx_other + dy_other*dy_other)
                            in_inner = 1;
                        else
                            in_inner = 0;
                    }
                } else { //If it is not a segment intersection, the last possibility corresponds to
                  //one of both current segments, intersecting a different segment of the other rectangle

                    point2D<T> *p1aux, *p2aux;
                    int aux_index;

                    //Check for intersection with inner rectangle first
                    aux_index = (i_inner + 1)%n_pinner;
                    p2aux = POLYGON_NTH_POINT(p_inner, aux_index);
                    for(j=0; j < n_pinner - 1; j++, aux_index = (aux_index + 1)%n_pinner) {
                        p1aux = p2aux;
                        p2aux = POLYGON_NTH_POINT(p_inner, (aux_index + 1)%n_pinner);
                        //If no intersection found, pass to the next segment
                        if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol2, p2pol2, p1aux, p2aux)) == NULL )
                            continue;
                        //Set the index for the first segment to analyse for the other rectangle
                        i_inner = aux_index;
                        allocatedResults[num_allocated++] = icurrent;
                        break;
                    }

                    if(icurrent == NULL) {  //If no intersection found for other rectangle segment, intersection must be
                                            //between the current segment from inner rectangle and one of the other rectangle

                        //Check for intersection with other rectangle
                        aux_index = (i_other + 1)%n_pother;
                        p2aux = POLYGON_NTH_POINT(p_other, aux_index);
                        for(j=0; j < n_pother - 1; j++, aux_index = (aux_index + 1)%n_pother) {
                            p1aux = p2aux;
                            p2aux = POLYGON_NTH_POINT(p_other, (aux_index + 1)%n_pother);
                            //If no intersection found, pass to the next segment
                            if( (icurrent = point2D<T>::getPreciseSegmentsIntersection(p1pol1, p2pol1, p1aux, p2aux)) == NULL )
                                continue;
                            //Set the index for the first segment to analyse for the other rectangle
                            i_other = aux_index;
                            allocatedResults[num_allocated++] = icurrent;
                            break;
                        }

                        if(icurrent == NULL) {  //If no intersection found for other rectangle segment, intersection must be
                                                //between the current segment from inner rectangle and one of the other rectangle
                            //An error???? It should not happen because if next point is out an intersection should exist.
                            //If an error of this type occurs, finish computation and give the partial intersection found.
                            std::cout << "Error of intersection not found!!" << std::endl;
                            break;
                        } else { //A point intersection for inner rectangle segment, so go to other rectangle
                            intersection_polygon->points[numIntersections  ].x = icurrent->x1;
                            intersection_polygon->points[numIntersections++].y = icurrent->y1;
                            i_inner = (i_inner + 1)%n_pinner;
                            in_inner = 0;
                        }
                    } else { //A point intersection for other rectangle segment, so go to inner rectangle
                        intersection_polygon->points[numIntersections  ].x = icurrent->x1;
                        intersection_polygon->points[numIntersections++].y = icurrent->y1;
                        i_other = (i_other + 1)%n_pother;
                        in_inner = 1;
                    }
                }
            }
        }
    } while (differentLastToAdd);

    for(i=0; i<num_allocated; i++)
        delete allocatedResults[i];

    if(numIntersections < 3) { //If the intersection can not form an area, the area is NULL
        delete intersection_polygon;
        return NULL;
    }

    return intersection_polygon;
}

template <typename T>
bool polygon2D<T>::memmove(int to, int from, int number) {
    if(to < 0 || from < 0) {
        std::cout << "polygon2D: memmove error: to or from lower than 0." << std::endl;
        return false;
    }

    if(to == from)
        return true;

    if(to < from) {
        if(number > points.size() - from) {
            std::cout << "polygon2D: memmove error: according to 'from' and 'number', elements to move exceding existing elements." << std::endl;
            return false;
        }
        for(int i = 0; i < number; i++)
            points[to + i] = points[from + i];
    }

    //from > to
    if(to + number > points.capacity())
        points.reserve(to + number);
    for(int i = number - 1; i >= 0; i--)
        points[to + i] = points[from + i];

    return true;
}


//Function which returns a polygon representing the area of intersection of two rectangles (with any orientation)
//TO DO: Generalize for any polygon
//This function presents problems with certain rectangle configurations due to floating point precision
//with segment intersections. When this occurs, memory leaks can happen.
template <typename T>
polygon2D<T> *polygon2D<T>::rectanglesIntersection(polygon2D<T> *rectangle1, polygon2D<T> *rectangle2) {

    //First check polygons bounding boxes, if there is no bounding boxes intersection, there is no polygon intersection
    if(!Rectangle<T>::thereIsIntersection(&POLYGON_2D_BB(rectangle1), &POLYGON_2D_BB(rectangle2)))
        return NULL;

    if(polygon2D<T>::equalPolygons(rectangle1, rectangle2, 0.001))
        return rectangle1->copy();

    int i;
    int num_inner_pol1 = 0, first_inner_pol1 = -1, num_inner_pol2 = 0, first_inner_pol2 = -1;
    point2D<T> *p1pol1, p1pol2;

    //Set inner point flags for both rectangles. An inner point can not be an intersecting point
    for(i=0; i<4; i++) {
        p1pol1 = POLYGON_NTH_POINT(rectangle1, i);
        if(rectangle2->pointInConvexPolygon(p1pol1->x, p1pol1->y, true)) {
            num_inner_pol1++;
            if(first_inner_pol1 < 0)
                first_inner_pol1 = i;
        }

        p1pol2 = POLYGON_NTH_POINT(rectangle2, i);
        if(rectangle1->pointInConvexPolygon(p1pol2->x, p1pol2->y, true)) {
            num_inner_pol2++;
            if(first_inner_pol2 < 0)
                first_inner_pol2 = i;
        }
    }

    //   std::cout  << "First Inn Pol 1: " << first_inner_pol1  << "\tFirst Inn Pol 2: " << first_inner_pol2 << std::endl;
    if(    fabs(rectangle1->points[0].x + 284.314450444592) < MAX_FLOAT_ERROR
        && fabs(rectangle2->points[0].x + 239.497076034818) < MAX_FLOAT_ERROR)
        std::cout << "Point found: x[0] = " << rectangle1->points[0].x << std::endl;
    if(num_inner_pol1 == 4)
        return rectangle1->copy();

    if(num_inner_pol2 == 4)
        return rectangle2->copy();

    Intersection<T> *intersectionsRectangle2[4][4], currentIntersections[4], allocatedResults[12], current;
    int j, k, l, with_segment_intersection, numIntersections, numTotalDoubles = 0, num_allocated = 0;
    int sequenceMap[12], //values: 0 (polygon point), 1 (intersection point), -1 (both)
        originalSegmentMap[12], //values: 0 (polygon point), 1 (intersection point), -1 (both)
        num_intersectionsRectangle2[4], segmentIntersectionRectangle1[4], segmentIntersectionRectangle2[4];
    point2D<T> *p2pol1, p2pol2;
    double dpol, dx, dy, dcur1, dcur2;
    bool move_inner_undone = true;
    polygon2D<T> *rectangle1_ext = copy_fpolygon2d_extended_size(rectangle1, 12);

    memset(sequenceMap, 0, 12*sizeof(int));
    memset(originalSegmentMap, 0, 12*sizeof(int));
    memset(num_intersectionsRectangle2, 0, 4*sizeof(int));
    memset(segmentIntersectionRectangle1, 0, 4*sizeof(int));
    memset(segmentIntersectionRectangle2, 0, 4*sizeof(int));

    p2pol1 = POLYGON_NTH_POINT(rectangle1, 0);

    for(i=0, j=1; i<4; i++) {
        p1pol1 = p2pol1;
        p2pol1 = POLYGON_NTH_POINT(rectangle1, (i + 1)%4);

        numIntersections = 0;
        with_segment_intersection = 0;

        p2pol2 = POLYGON_NTH_POINT(rectangle2, 0);
        for(k=0; k<4; k++) {
            p1pol2 = p2pol2;
            p2pol2 = POLYGON_NTH_POINT(rectangle2, (k + 1)%4);

            //If no intersection found, pass to the next pair
            if( (current = get_segments_intersection(p1pol1, p2pol1, p1pol2, p2pol2)) == NULL )
                continue;

            allocatedResults[num_allocated++] = current;

            if(current->type == 0) { //If the intersection corresponds to a segment, for two rectangles
                                     //the other intersections MUST correspond to the same limit points of the
                                     //resulting segments
                if(++numTotalDoubles == 4) { //If this counter arrives to 4, it means that there is a perfect match between rectangles
                    for(l=0; l<num_allocated; l++)
                        delete allocatedResults[l];
                    delete rectangle1_ext;
                    return copy_fpolygon2d(rectangle1);
                }
                currentIntersections[0] = intersectionsRectangle2[k][0] = current;
                segmentIntersectionRectangle1[i] = 1;
                with_segment_intersection = segmentIntersectionRectangle2[k] = num_intersectionsRectangle2[k] = numIntersections = 1;
                break;
            }

            if(!segmentIntersectionRectangle2[k])
                intersectionsRectangle2[k][num_intersectionsRectangle2[k]++] = current;
            currentIntersections[numIntersections++] = current;
        }

        if(numIntersections > 0) {     //Process currently found intersections

            dx = p1pol1->x - p2pol1->x;
            dy = p1pol1->y - p2pol1->y;
            dpol = dx*dx + dy*dy;

            if(with_segment_intersection) { //Treat special case of a segment intersection result

                dx = currentIntersections[0]->x1 - p1pol1->x;
                dy = currentIntersections[0]->y1 - p1pol1->y;
                dcur1 = dx*dx + dy*dy;
                dx = currentIntersections[0]->x2 - p1pol1->x;
                dy = currentIntersections[0]->y2 - p1pol1->y;
                dcur2 = dx*dx + dy*dy;

                if(dcur1 < dcur2) {
                    if(dcur1 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
                        rectangle1_ext->n_points++;
                        if(i < 3)
                            memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
                            rectangle1_ext->points[j].x = currentIntersections[0]->x1;
                            rectangle1_ext->points[j].y = currentIntersections[0]->y1;
                            originalSegmentMap[j] = i;
                            sequenceMap[j++] = 1;
                    } else {
                        originalSegmentMap[j-1] = i;
                        sequenceMap[j-1] = -1;
                    }
                    if( fabs(dpol - dcur2) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
                        rectangle1_ext->n_points++;
                        if(i < 3)
                            memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
                        rectangle1_ext->points[j].x = currentIntersections[0]->x2;
                        rectangle1_ext->points[j].y = currentIntersections[0]->y2;
                        originalSegmentMap[j] = i;
                        sequenceMap[j++] = 1;
                    } else {
                        originalSegmentMap[j] = i;
                        sequenceMap[j] = -1;
                    }
                } else { //Same idea but with inverse order
                    if(dcur2 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
                        rectangle1_ext->n_points++;
                        if(i < 3)
                            memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
                        rectangle1_ext->points[j].x = currentIntersections[0]->x2;
                        rectangle1_ext->points[j].y = currentIntersections[0]->y2;
                        originalSegmentMap[j] = i;
                        sequenceMap[j++] = 1;
                    } else {
                        originalSegmentMap[j-1] = i;
                        sequenceMap[j-1] = -1;
                    }

                    if( fabs(dpol - dcur1) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
                        rectangle1_ext->n_points++;
                        if(i < 3)
                            memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
                        rectangle1_ext->points[j].x = currentIntersections[0]->x1;
                        rectangle1_ext->points[j].y = currentIntersections[0]->y1;
                        originalSegmentMap[j] = i;
                        sequenceMap[j++] = 1;
                    } else {
                        originalSegmentMap[j] = i;
                        sequenceMap[j] = -1;
                    }
                }
            } else { //Point intersections: For two rectangles, only a maximum of two intersections can happen in a rectangle segment
                     //TO DO: For generalizing to polygons, intersection points must be ordered according to distance
                     //As for rectangles (and for convex polygons) the maximum number of intersections is 2, the ordering
                     //is made by cases

                //If there is two intersections or more, verify repeated points (case where a corner point intersects one of the other rectangle )
                if(numIntersections >= 2) {
                    for(k=0; k<numIntersections-1; k++)
                        for(l=k+1; l<numIntersections; l++) {
                            if(    fabs(currentIntersections[k]->x1 - currentIntersections[l]->x1) < MAX_FLOAT_ERROR
                                && fabs(currentIntersections[k]->y1 - currentIntersections[l]->y1) < MAX_FLOAT_ERROR ) {
                                if(l + 1 < numIntersections)
                                    memmove(currentIntersections + l, currentIntersections + l + 1, sizeof(Intersection<T> *)*(numIntersections-l-1) );
                                numIntersections--;
                                break;
                            }
                        }
                }

                if(numIntersections == 2) {
                    dx = currentIntersections[0]->x1 - p1pol1->x;
                    dy = currentIntersections[0]->y1 - p1pol1->y;
                    dcur1 = dx*dx + dy*dy;
                    dx = currentIntersections[1]->x1 - p1pol1->x;
                    dy = currentIntersections[1]->y1 - p1pol1->y;
                    dcur2 = dx*dx + dy*dy;

                    if(dcur1 < dcur2) {
                        if(dcur1 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
                            rectangle1_ext->n_points++;
               if(i < 3)
                 memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );

               rectangle1_ext->points[j].x = currentIntersections[0]->x1;
               rectangle1_ext->points[j].y = currentIntersections[0]->y1;

               originalSegmentMap[j] = i;
               sequenceMap[j++] = 1;

             } else {
               originalSegmentMap[j-1] = i;
               sequenceMap[j-1] = -1;
             }
             if( fabs(dpol - dcur2) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
               rectangle1_ext->n_points++;
               if(i<3)
                 memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle1_ext->points[j].x = currentIntersections[1]->x1;
               rectangle1_ext->points[j].y = currentIntersections[1]->y1;
               originalSegmentMap[j] = i;
               sequenceMap[j++] = 1;
             } else {
               originalSegmentMap[j] = i;
               sequenceMap[j] = -1;
             }
           } else { //Same idea but with inverse order
             if(dcur2 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
               rectangle1_ext->n_points++;
               if(i<3)
                 memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle1_ext->points[j].x = currentIntersections[1]->x1;
               rectangle1_ext->points[j].y = currentIntersections[1]->y1;
               originalSegmentMap[j] = i;
               sequenceMap[j++] = 1;
             } else {
               originalSegmentMap[j-1] = i;
               sequenceMap[j-1] = -1;
             }
             if( fabs(dpol - dcur1) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
               rectangle1_ext->n_points++;
               if(i<3)
                 memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle1_ext->points[j].x = currentIntersections[0]->x1;
               rectangle1_ext->points[j].y = currentIntersections[0]->y1;
               originalSegmentMap[j] = i;
               sequenceMap[j++] = 1;
             } else {
               originalSegmentMap[j] = i;
               sequenceMap[j] = -1;
             }
           }
         } else { //Just one intersection
           dx = currentIntersections[0]->x1 - p1pol1->x;
           dy = currentIntersections[0]->y1 - p1pol1->y;
           dcur1 = dx*dx + dy*dy;

           if(dcur1 > MAX_FLOAT_ERROR && fabs(dpol - dcur1) > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
             rectangle1_ext->n_points++;
             if(i<3)
               memmove(rectangle1_ext->points + j + 1, rectangle1_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle1_ext->points[j].x = currentIntersections[0]->x1;
             rectangle1_ext->points[j].y = currentIntersections[0]->y1;
             originalSegmentMap[j] = i;
             sequenceMap[j++] = 1;
           } else if( fabs(dpol - dcur1) < MAX_FLOAT_ERROR ) {
             originalSegmentMap[j] = i;
             sequenceMap[j] = -1;
           } else {
             originalSegmentMap[j-1] = i;
             sequenceMap[j-1] = -1;
           }
         }
       }
     }

     if(i < 3) {
       if(move_inner_undone && first_inner_pol1 > 0 && first_inner_pol1 == i + 1) {
         first_inner_pol1 = j;
         move_inner_undone = false;
       }
       j++;
     }
   }

   int numTotalIntersections = 0;
   for(i=0; i<rectangle1_ext->n_points; i++)
     if(sequenceMap[i] != 0)
       numTotalIntersections++;

   //If no inner points for both rectangles, there is two possibilities
   if( num_inner_pol1 == 0 && num_inner_pol2 == 0 ) {
     if(numTotalIntersections < 3) { //If the intersection can not form an area, the area is NULL
       for(l=0; l<num_allocated; l++)
         delete allocatedResults[l];
       delete rectangle1_ext;
       return NULL;
     }

     for(i=0; i<num_allocated; i++)
       delete allocatedResults[i];

     //Else, the intersection is just formed by the intersection points
     for(i=0, j=0; i<rectangle1_ext->n_points; i++)
       if(sequenceMap[i] != 0) {
         rectangle1_ext->points[j].x = rectangle1_ext->points[i].x;
         rectangle1_ext->points[j].y = rectangle1_ext->points[i].y;
         j++;
       }
     rectangle1_ext->n_points = numTotalIntersections;
     return rectangle1_ext;

   }

   //   std::cout << "Extended Rectangle 1:" << std::endl;
   //for(i=0; i < rectangle1_ext->n_points; i++)
   //  std::cout << "("   << rectangle1_ext->points[i].x  << ", " << rectangle1_ext->points[i].y << ") ";
   //std::cout  << std::endl;

   //If the execution arrives here, then is necessary to also generate the extended polygon for second rectangle

   int sequenceMap2[12], originalSegmentMap2[12];
   polygon2D<T> *rectangle2_ext = copy_fpolygon2d_extended_size(rectangle2, 12);

   memset(sequenceMap2, 0, 12*sizeof(int));
   memset(originalSegmentMap2, 0, 12*sizeof(int));

   move_inner_undone = true;
   p2pol2 = POLYGON_NTH_POINT(rectangle2, 0);

   for(i=0, j=1; i<4; i++) {

     p1pol2 = p2pol2;
     p2pol2 = POLYGON_NTH_POINT(rectangle2, (i + 1)%4);

     if(num_intersectionsRectangle2[i] > 0) {     //Process currently found intersections

       dx = p1pol2->x - p2pol2->x;
       dy = p1pol2->y - p2pol2->y;
       dpol = dx*dx + dy*dy;

       if(segmentIntersectionRectangle2[i]) { //Treat special case of a segment intersection result

         dx = intersectionsRectangle2[i][0]->x1 - p1pol2->x;
         dy = intersectionsRectangle2[i][0]->y1 - p1pol2->y;
         dcur1 = dx*dx + dy*dy;
         dx = intersectionsRectangle2[i][0]->x2 - p1pol2->x;
         dy = intersectionsRectangle2[i][0]->y2 - p1pol2->y;
         dcur2 = dx*dx + dy*dy;

         if(dcur1 < dcur2) {
           if(dcur1 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
             rectangle2_ext->n_points++;
             if(i<3)
               memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x1;
             rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y1;
             originalSegmentMap2[j] = i;
             sequenceMap2[j++] = 1;
           } else {
             originalSegmentMap2[j-1] = i;
             sequenceMap2[j-1] = -1;
           }
           if( fabs(dpol - dcur2) >= MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
             rectangle2_ext->n_points++;
             if(i<3)
               memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x2;
             rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y2;
             originalSegmentMap2[j] = i;
             sequenceMap2[j++] = 1;
           } else {
             originalSegmentMap2[j] = i;
             sequenceMap2[j] = -1;
           }
         } else { //Same idea but with inverse order
           if(dcur2 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
             rectangle2_ext->n_points++;
             if(i<3)
               memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x2;
             rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y2;
             originalSegmentMap2[j] = i;
             sequenceMap2[j++] = 1;
           } else {
             originalSegmentMap2[j-1] = i;
             sequenceMap2[j-1] = -1;
           }
           if( fabs(dpol - dcur1) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
             rectangle2_ext->n_points++;
             if(i<3)
               memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x1;
             rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y1;
             originalSegmentMap2[j] = i;
             sequenceMap2[j++] = 1;
           } else {
             originalSegmentMap2[j] = i;
             sequenceMap2[j] = -1;
           }
         }
       } else { //Point intersections: For two rectangles, only a maximum of two intersections can happen in a rectangle segment
                //TO DO: For generalizing to polygons, intersection points must be ordered according to distance
                //As for rectangles (and for convex polygons) the maximum number of intersections is 2, the ordering
                //is made by cases
         if(num_intersectionsRectangle2[i] >= 2) {
           for(k=0; k<num_intersectionsRectangle2[i]-1; k++)
             for(l=k+1; l<num_intersectionsRectangle2[i]; l++) {
               if(    fabs(intersectionsRectangle2[i][k]->x1 - intersectionsRectangle2[i][l]->x1) < MAX_FLOAT_ERROR
                   && fabs(intersectionsRectangle2[i][k]->y1 - intersectionsRectangle2[i][l]->y1) < MAX_FLOAT_ERROR ) {
                 if(l+1 < num_intersectionsRectangle2[i])
                   memmove(intersectionsRectangle2[i] + l, intersectionsRectangle2[i] + l + 1, sizeof(Intersection<T> *)*(num_intersectionsRectangle2[i]-l-1) );
                 num_intersectionsRectangle2[i]--;
                 break;
               }
             }
         }

         if(num_intersectionsRectangle2[i] == 2) { //se
           dx = intersectionsRectangle2[i][0]->x1 - p1pol2->x;
           dy = intersectionsRectangle2[i][0]->y1 - p1pol2->y;
           dcur1 = dx*dx + dy*dy;
           dx = intersectionsRectangle2[i][1]->x1 - p1pol2->x;
           dy = intersectionsRectangle2[i][1]->y1 - p1pol2->y;
           dcur2 = dx*dx + dy*dy;

           if(dcur1 < dcur2) {
             if(dcur1 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
               rectangle2_ext->n_points++;
               if(i<3)
                 memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x1;
               rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y1;
               originalSegmentMap2[j] = i;
               sequenceMap2[j++] = 1;
             } else {
               originalSegmentMap2[j-1] = i;
               sequenceMap2[j-1] = -1;
             }
             if( fabs(dpol - dcur2) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
               rectangle2_ext->n_points++;
               if(i<3)
                 memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle2_ext->points[j].x = intersectionsRectangle2[i][1]->x1;
               rectangle2_ext->points[j].y = intersectionsRectangle2[i][1]->y1;
               originalSegmentMap2[j] = i;
               sequenceMap2[j++] = 1;
             } else {
               originalSegmentMap2[j] = i;
               sequenceMap2[j] = -1;
             }
           } else { //Same idea but with inverse order
             if(dcur2 > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
               rectangle2_ext->n_points++;
               if(i<3)
                 memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle2_ext->points[j].x = intersectionsRectangle2[i][1]->x1;
               rectangle2_ext->points[j].y = intersectionsRectangle2[i][1]->y1;
               originalSegmentMap2[j] = i;
               sequenceMap2[j++] = 1;
             } else {
               originalSegmentMap2[j-1] = i;
               sequenceMap2[j-1] = -1;
             }

             if( fabs(dpol - dcur1) > MAX_FLOAT_ERROR ) { //If the farest segment point is not the other rectangle point, the point is added
               rectangle2_ext->n_points++;
               if(i<3)
                 memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
               rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x1;
               rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y1;
               originalSegmentMap2[j] = i;
               sequenceMap2[j++] = 1;
             } else {
               originalSegmentMap2[j] = i;
               sequenceMap2[j] = -1;
             }
           }
         } else { //Just one intersection
           dx = intersectionsRectangle2[i][0]->x1 - p1pol2->x;
           dy = intersectionsRectangle2[i][0]->y1 - p1pol2->y;
           dcur1 = dx*dx + dy*dy;

           if(dcur1 > MAX_FLOAT_ERROR && fabs(dpol - dcur1) > MAX_FLOAT_ERROR) { //If the nearest segment point is not the rectangle point itself, the point is added
             rectangle2_ext->n_points++;
             if(i<3)
               memmove(rectangle2_ext->points + j + 1, rectangle2_ext->points + j, sizeof(struct fpoint_2d)*(3-i) );
             rectangle2_ext->points[j].x = intersectionsRectangle2[i][0]->x1;
             rectangle2_ext->points[j].y = intersectionsRectangle2[i][0]->y1;
             originalSegmentMap2[j] = i;
             sequenceMap2[j++] = 1;
           } else if( fabs(dcur1 - dpol) < MAX_FLOAT_ERROR ) {
             originalSegmentMap2[j] = i;
             sequenceMap2[j] = -1;
           } else {
             originalSegmentMap2[j] = i;
             sequenceMap2[j-1] = -1;
           }
         }
       }
     }

     if(i < 3) {
       if(move_inner_undone && first_inner_pol2 > 0 && first_inner_pol2 == i + 1) {
         first_inner_pol2 = j;
         move_inner_undone = false;
       }
       j++;
     }

   }

   //   std::cout << "Extended Rectangle 2:\t" << std::endl;
   //for(i=0; i < rectangle2_ext->n_points; i++)
   //  std::cout << "("   << rectangle2_ext->points[i].x  << ", " << rectangle2_ext->points[i].y << ") ";
   //std::cout  << std::endl;

   //std::cout  << "First Inn Pol 1: " << first_inner_pol1  << "\tFirst Inn Pol 2: " << first_inner_pol2 << std::endl;

   //Now get the intersection polygon
   polygon2D<T> *inner_point_rectangle, the_other_rectangle, intersection_polygon;
   int *innerSequenceMap, *otherSequenceMap, *innerOriginalSegmentMap, *otherOriginalSegmentMap, *innerSegmentIntersection, *otherSegmentIntersection;
   int first_inner_index, first_other_index, inner_index, other_index, nother, ninner, search_first_other = 1, current_inner = 1;

   //Because of previous validation it MUST exist an inner point
   if(num_inner_pol1 > 0) {
     inner_point_rectangle = rectangle1_ext;
     the_other_rectangle = rectangle2_ext;
     first_inner_index = inner_index = first_inner_pol1;
     innerSequenceMap = sequenceMap;
     otherSequenceMap = sequenceMap2;
     innerOriginalSegmentMap = originalSegmentMap;
     otherOriginalSegmentMap = originalSegmentMap2;
     innerSegmentIntersection = segmentIntersectionRectangle1;
     otherSegmentIntersection = segmentIntersectionRectangle2;
   } else {
     inner_point_rectangle = rectangle2_ext;
     the_other_rectangle = rectangle1_ext;
     first_inner_index = inner_index = first_inner_pol2;
     innerSequenceMap = sequenceMap2;
     otherSequenceMap = sequenceMap;
     innerOriginalSegmentMap = originalSegmentMap2;
     otherOriginalSegmentMap = originalSegmentMap;
     innerSegmentIntersection = segmentIntersectionRectangle2;
     otherSegmentIntersection = segmentIntersectionRectangle1;
   }

   ninner = inner_point_rectangle->n_points;
   nother = the_other_rectangle->n_points;

   //Construct the intersection polygon
   intersection_polygon = copy_fpolygon2d(inner_point_rectangle);

   for(i=0, j=0, k=0; i<ninner || j<nother; ) {
     if(k>ninner) {
       int l;
       std::cout.precision(15);
       std::cout << "Rectangle 1:\t" << std::endl;
       for(l=0; l < rectangle1->n_points; l++)
         std::cout << "("   << rectangle1->points[l].x  << ", " << rectangle1->points[l].y << ") ";
       std::cout  << std::endl;
       std::cout << "Rectangle 2:\t" << std::endl;
       for(l=0; l < rectangle2->n_points; l++)
         std::cout << "("   << rectangle2->points[l].x  << ", " << rectangle2->points[l].y << ") ";
       std::cout  << std::endl;
       std::cout << "Extended Rectangle 1:\t" << std::endl;
       for(l=0; l < rectangle1_ext->n_points; l++)
         std::cout << "("   << rectangle1_ext->points[l].x  << ", " << rectangle1_ext->points[l].y << ") ";
       std::cout  << std::endl;
       std::cout << "Extended Rectangle 2:\t" << std::endl;
       for(l=0; l < rectangle2_ext->n_points; l++)
         std::cout << "("   << rectangle2_ext->points[l].x  << ", " << rectangle2_ext->points[l].y << ") ";
       std::cout  << std::endl;
     }

     if(current_inner) { //If currently processing inner rectangle points
       //if point is intersection and it does not belong to a segment intersection, swap from inner to the other sequence
       if(   innerSequenceMap[inner_index] != 0                   //Check if point corresponds to an intersection
          && innerSegmentIntersection[innerOriginalSegmentMap[inner_index]] == 0           //Check if point is not the first of a segment intersection
          && innerSegmentIntersection[innerOriginalSegmentMap[(inner_index + 3)%4]] == 0 ) //Check if point was not the second of a segment intersection
         current_inner = 0;

       //If the first swap between polygons does not happen yet, nothing more has to be done as the intersection
       //polygon was initialized with the inner point rectangle. Just increment the index of point added.
       if(search_first_other) {
         intersection_polygon->points[k  ].x = inner_point_rectangle->points[inner_index].x;
         intersection_polygon->points[k++].y = inner_point_rectangle->points[inner_index].y;

         if(current_inner == 0) { //If the first swap will happen, search for the first index of the other sequence
           search_first_other = 0;
           for(l=0; l<nother; l++)
             if(    fabs(inner_point_rectangle->points[inner_index].x - the_other_rectangle->points[l].x) < MAX_FLOAT_ERROR
                 && fabs(inner_point_rectangle->points[inner_index].y - the_other_rectangle->points[l].y) < MAX_FLOAT_ERROR ) {
               first_other_index = other_index = l;
               break;
             }
           other_index = (other_index + 1) % nother;
           j++;
         }
       } else { //If the first swap has been already made, the point has to be added, because information from the
         //other rectangle have been added, altering maybe the order of the inner point rectangle
         intersection_polygon->points[k  ].x = inner_point_rectangle->points[inner_index].x;
         intersection_polygon->points[k++].y = inner_point_rectangle->points[inner_index].y;

         if(current_inner == 0) { //Search for the next index of the other sequence
           //Search for next intersection point until find it or until the sequence is finished
           while(    j < nother
                  && (    fabs(inner_point_rectangle->points[inner_index].x - the_other_rectangle->points[other_index].x) >= MAX_FLOAT_ERROR
                       || fabs(inner_point_rectangle->points[inner_index].y - the_other_rectangle->points[other_index].y) >= MAX_FLOAT_ERROR ) ) {
             other_index = (other_index + 1) % nother;
             j++;
           }

           if(j < nother) { //If index is still valid, advance to the next position, because the intersection point has been
             j++;           //already added
             other_index = (other_index + 1) % nother;
           }
         }
       }

       inner_index = (inner_index + 1) % ninner;
       i++;

       //If we have made the loop with this rectangle and there is no swap, we have finished
       if(current_inner == 1 && inner_index == first_inner_index)
         break;

     } else { //If now the other sequence is active

       //if point is intersection and it does not belong to a segment intersection, swap from inner to the other sequence
       if(   otherSequenceMap[other_index] != 0                   //Check if point corresponds to an intersection
          && otherSegmentIntersection[otherOriginalSegmentMap[other_index]] == 0           //Check if point is not the first of a segment intersection
          && otherSegmentIntersection[otherOriginalSegmentMap[(other_index + 3) % nother]] == 0 ) //Check if point was not the second of a segment intersection
         current_inner = 1;

       intersection_polygon->points[k  ].x = the_other_rectangle->points[other_index].x;
       intersection_polygon->points[k++].y = the_other_rectangle->points[other_index].y;

       if(current_inner == 1) { //Search for the next index of the inner sequence
         //Search for next intersection point until find it or until the sequence is finished
         while(    i < ninner
                   && (    fabs(inner_point_rectangle->points[inner_index].x - the_other_rectangle->points[other_index].x) >= MAX_FLOAT_ERROR
                        || fabs(inner_point_rectangle->points[inner_index].y - the_other_rectangle->points[other_index].y) >= MAX_FLOAT_ERROR ) ) {
           inner_index = (inner_index + 1) % ninner;
           i++;
         }

         i++;
         if(i < ninner) { //If index is still valid, advance to the next position, because the intersection point has been
                          //already added
           inner_index = (inner_index + 1) % ninner;
         } else //We have arrived to the same initial point
           break;
       }

       other_index = (other_index + 1) % nother;
       j++;

       //If we have made the loop with this rectangle and there is no swap, we have finished
       if(current_inner == 0 && other_index == first_other_index)
         break;
     }
   }

   intersection_polygon->n_points = k;

   for(l=0; l<num_allocated; l++)
     delete allocatedResults[l];

   delete rectangle1_ext;
   delete rectangle2_ext;

   return intersection_polygon;

 }

//Calculate de intersection between to line segments. The function returns a structure which represent the two
//result possibilities: 1 point, 1 segment represented by its two limiting points
template <typename T>
Intersection<T> *point2D<T>::getSegmentsIntersection(point2D<T> *p1L1, point2D<T> *p2L1, point2D<T> *p1L2, point2D<T> *p2L2) {
   double
     xp1L1 = POINT_2D_X(p1L1), yp1L1 = POINT_2D_Y(p1L1),
     xp2L1 = POINT_2D_X(p2L1), yp2L1 = POINT_2D_Y(p2L1),
     xp1L2 = POINT_2D_X(p1L2), yp1L2 = POINT_2D_Y(p1L2),
     xp2L2 = POINT_2D_X(p2L2), yp2L2 = POINT_2D_Y(p2L2),
     minxL1, minxL2, minyL1, minyL2, maxxL1, maxxL2, maxyL1, maxyL2;

   if(xp1L1 < xp2L1) { minxL1 = xp1L1; maxxL1 = xp2L1; }
   else              { minxL1 = xp2L1; maxxL1 = xp1L1; }

   if(yp1L1 < yp2L1) { minyL1 = yp1L1; maxyL1 = yp2L1; }
   else              { minyL1 = yp2L1; maxyL1 = yp1L1; }

   if(xp1L2 < xp2L2) { minxL2 = xp1L2; maxxL2 = xp2L2; }
   else              { minxL2 = xp2L2; maxxL2 = xp1L2; }

   if(yp1L2 < yp2L2) { minyL2 = yp1L2; maxyL2 = yp2L2; }
   else              { minyL2 = yp2L2; maxyL2 = yp1L2; }

   //X coordinate criteria
   if( minxL1 > maxxL2 || maxxL1 < minxL2)
     return NULL;

   //Y coordinate criteria
   if( minyL1 > maxyL2 || maxyL1 < minyL2)
     return NULL;


   if( fabs(xp1L1 - xp2L1) < MAX_FLOAT_ERROR ) { //Infinite slope for L1
     if( fabs(xp1L2 - xp2L2) < MAX_FLOAT_ERROR ) { //Infinite slope for L2 also
       if( fabs(xp1L1 - xp1L2) < MAX_FLOAT_ERROR ) { //Intersection of parallel lines
         Intersection<T> *result = new Intersection<T>();
         double
           min = minyL1 > minyL2 ? minyL1 : minyL2,
           max = maxyL1 < maxyL2 ? maxyL1 : maxyL2;
         if ( fabs(min - max) < MAX_FLOAT_ERROR ) { //If intersection occurs just in a limit point
           result->x1 = xp1L1; result->y1 = min; result->type = 1;
         } else { //If intersection result in a segment, extreme values are given and intersection type is 0
           result->x1 = xp1L1; result->y1 = min;
           result->x2 = xp1L1; result->y2 = max;
           result->type = 0;
         }
         return result;
       }
       return NULL;
     }

     //If L2 has a non infinite slope:
     double //the valid interval for y is between the maximal minimum, and the minimal maximum
       min = minyL1 > minyL2 ? minyL1 : minyL2,
       max = maxyL1 < maxyL2 ? maxyL1 : maxyL2,
       x = xp1L1,
       y = ((xp1L1 - xp2L2)*yp1L2 - (xp1L1 - xp1L2)*yp2L2)/(xp1L2 - xp2L2);

     if( y >= min && max >= y ) {
         Intersection<T> *result = new Intersection<T>();
       result->x1 = x;
       result->y1 = y;
       result->type = 1;
       return result;
     } else
       return NULL;
   } else if( fabs(xp1L2 - xp2L2) < MAX_FLOAT_ERROR ) { //Infinite slope for L2, and finite slope for L1

     double //the valid interval for y is between the maximal minimum, and the minimal maximum
       min = minyL1 > minyL2 ? minyL1 : minyL2,
       max = maxyL1 < maxyL2 ? maxyL1 : maxyL2,
       x = xp1L2,
       y = ((xp1L2 - xp2L1)*yp1L1 - (xp1L2 - xp1L1)*yp2L1)/(xp1L1 - xp2L1);

     if( y >= min && max >= y ) {
         Intersection<T> *result = new Intersection<T>();
       result->x1 = x;
       result->y1 = y;
       result->type = 1;
       return result;
     } else
       return NULL;

   }

   //If both slopes are finite
   double
     a1 = (yp2L1 - yp1L1)/(xp2L1 - xp1L1),
     a2 = (yp2L2 - yp1L2)/(xp2L2 - xp1L2),
     b1 = yp2L1 - a1*xp2L1,
     b2 = yp2L2 - a2*xp2L2;

   //Parallel lines
   if( fabs(a1 - a2) < MAX_FLOAT_ERROR_LITTLE_NUMBERS ) {

     //If intercepts are different, there is no intersection between the parallel lines
     if ( fabs(b1 - b2) >= MAX_FLOAT_ERROR )
       return NULL;

     //If there is intersection, calculate
         Intersection<T> *result = new Intersection<T>();
     double
           min = minxL1 > minxL2 ? minxL1 : minxL2,
           max = maxxL1 < maxxL2 ? maxxL1 : maxxL2;
     if ( fabs(min - max) < MAX_FLOAT_ERROR ) { //If intersection occurs just in a limit point
       result->x1 = min; result->y1 = min*a1 + b1; result->type = 1;
     } else { //If intersection result in a segment, extreme values are given and intersection type is 0
       result->x1 = min; result->y1 = min * a1 + b1;
       result->x2 = max; result->y2 = max * a1 + b1;
       result->type = 0;
     }
     return result;
    }


    double
     min = minxL1 > minxL2 ? minxL1 : minxL2,
     max = maxxL1 < maxxL2 ? maxxL1 : maxxL2,
     den = a1 - a2,
     x = (b2 - b1) / den,
     y = (b2*a1 - b1*a2) / den;
    if( x >= min && max >= x ) {
        Intersection<T> *result = new Intersection<T>();
     result->x1 = x;
     result->y1 = y;
     result->type = 1;
     return result;
   }

   return NULL;
}

//Calculate de intersection between to line segments. The function returns a structure which represent the two
//result possibilities: 1 point, 1 segment represented by its two limiting points
//This variant of get_segments_intersection does not consider floating point errors, giving the precise intersections.
//If floating point error is present it will be handled as the real point, avoiding suppositions about the real value
template <typename T>
Intersection<T> *point2D<T>::getPreciseSegmentsIntersection(point2D<T> *p1L1, point2D<T> *p2L1, point2D<T> *p1L2, point2D<T> *p2L2) {
    double
        xp1L1 = POINT_2D_X(p1L1), yp1L1 = POINT_2D_Y(p1L1),
        xp2L1 = POINT_2D_X(p2L1), yp2L1 = POINT_2D_Y(p2L1),
        xp1L2 = POINT_2D_X(p1L2), yp1L2 = POINT_2D_Y(p1L2),
        xp2L2 = POINT_2D_X(p2L2), yp2L2 = POINT_2D_Y(p2L2),
        minxL1, minxL2, minyL1, minyL2, maxxL1, maxxL2, maxyL1, maxyL2;

    if(xp1L1 < xp2L1) { minxL1 = xp1L1; maxxL1 = xp2L1; }
    else              { minxL1 = xp2L1; maxxL1 = xp1L1; }

    if(yp1L1 < yp2L1) { minyL1 = yp1L1; maxyL1 = yp2L1; }
    else              { minyL1 = yp2L1; maxyL1 = yp1L1; }

    if(xp1L2 < xp2L2) { minxL2 = xp1L2; maxxL2 = xp2L2; }
    else              { minxL2 = xp2L2; maxxL2 = xp1L2; }

    if(yp1L2 < yp2L2) { minyL2 = yp1L2; maxyL2 = yp2L2; }
    else              { minyL2 = yp2L2; maxyL2 = yp1L2; }

    //X coordinate criteria
    if( minxL1 > maxxL2 || maxxL1 < minxL2)
        return NULL;

    //Y coordinate criteria
    if( minyL1 > maxyL2 || maxyL1 < minyL2)
        return NULL;


    if( xp1L1 == xp2L1 ) { //Infinite slope for L1
        if( xp1L2 == xp2L2 ) { //Infinite slope for L2 also
            if( xp1L1 == xp1L2 ) { //Intersection of parallel lines
                Intersection<T> *result = new Intersection<T>();
                double
                    min = minyL1 > minyL2 ? minyL1 : minyL2,
                    max = maxyL1 < maxyL2 ? maxyL1 : maxyL2;
                if ( min == max ) { //If intersection occurs just in a limit point
                    result->x1 = xp1L1; result->y1 = min; result->type = 1;
                } else { //If intersection result in a segment, extreme values are given and intersection type is 0
                    result->x1 = xp1L1; result->y1 = min;
                    result->x2 = xp1L1; result->y2 = max;
                    result->type = 0;
                }
                return result;
            }
            return NULL;
        }

        //If L2 has a non infinite slope:
        double //the valid interval for y is between the maximal minimum, and the minimal maximum
            min = minyL1 > minyL2 ? minyL1 : minyL2,
            max = maxyL1 < maxyL2 ? maxyL1 : maxyL2,
            x = xp1L1,
            y = ((xp1L1 - xp2L2)*yp1L2 - (xp1L1 - xp1L2)*yp2L2)/(xp1L2 - xp2L2);

        if( y >= min && max >= y ) {
            Intersection<T> *result = new Intersection<T>();
            result->x1 = x;
            result->y1 = y;
            result->type = 1;
            return result;
        } else
            return NULL;
        } else if( xp1L2 == xp2L2 ) { //Infinite slope for L2, and finite slope for L1

            double //the valid interval for y is between the maximal minimum, and the minimal maximum
                min = minyL1 > minyL2 ? minyL1 : minyL2,
                max = maxyL1 < maxyL2 ? maxyL1 : maxyL2,
                x = xp1L2,
                y = ((xp1L2 - xp2L1)*yp1L1 - (xp1L2 - xp1L1)*yp2L1)/(xp1L1 - xp2L1);

            if( y >= min && max >= y ) {
                Intersection<T> *result = new Intersection<T>();
                result->x1 = x;
                result->y1 = y;
                result->type = 1;
                return result;
            } else
                return NULL;
   }

    //If both slopes are finite
    double
        a1 = (yp2L1 - yp1L1)/(xp2L1 - xp1L1),
        a2 = (yp2L2 - yp1L2)/(xp2L2 - xp1L2),
        b1 = yp2L1 - a1*xp2L1,
        b2 = yp2L2 - a2*xp2L2;

    //Parallel lines
    if(a1 == a2) {

        //If intercepts are different, there is no intersection between the parallel lines
        if ( b1 != b2)
            return NULL;

        //If there is intersection, calculate
        Intersection<T> *result = new Intersection<T>();
        double
            min = minxL1 > minxL2 ? minxL1 : minxL2,
            max = maxxL1 < maxxL2 ? maxxL1 : maxxL2;
        if ( min == max ) { //If intersection occurs just in a limit point
            result->x1 = min; result->y1 = min*a1 + b1; result->type = 1;
        } else { //If intersection result in a segment, extreme values are given and intersection type is 0
            result->x1 = min; result->y1 = min * a1 + b1;
            result->x2 = max; result->y2 = max * a1 + b1;
            result->type = 0;
        }
        return result;
    }

    double
        min = minxL1 > minxL2 ? minxL1 : minxL2,
        max = maxxL1 < maxxL2 ? maxxL1 : maxxL2,
        den = a1 - a2,
        x = (b2 - b1) / den,
        y = (b2*a1 - b1*a2) / den;
    if( x >= min && max >= x ) {
        Intersection<T> *result = new Intersection<T>();
        result->x1 = x;
        result->y1 = y;
        result->type = 1;
        return result;
    }

    return NULL;
}


template <typename T>
bool point2D<T>::pointPreciselyInSegmentStrict(double x, double y, point2D<T> *begin, point2D<T> *end) {

   double
        x1 = POINT_2D_X(begin),
        y1 = POINT_2D_Y(begin),
        x2 = POINT_2D_X(end),
        y2 = POINT_2D_Y(end),
        min, max;

   //Infinite slope
   if(x1 == x2) {
     if(y1 == y2) //A point
       return x == x1 && y == y1 ? 1 : 0;
     if(x != x1) //The only possible x = x1
       return 0;
     //Test y allowed interval
     if(y1 > y2) { min = y2; max = y1; }
     else        { min = y1; max = y2; }
     return y >= min && y <= max ? 1 : 0;
   }

   //Test x allowed interval
   if(x1 > x2) { min = x2; max = x1; }
   else        { min = x1; max = x2; }
   if( x < min || x > max )
     return 0;

   //Test y allowed interval
   if(y1 > y2) { min = y2; max = y1; }
   else        { min = y1; max = y2; }
   if( y < min || y > max )
     return 0;

   //Test if point belongs to the line (no devisions to avoid loss of float precision)
   return y*(x2 - x1) == x*(y2 - y1) + x2*y1 - x1*y2  ? 1 : 0;
 }

/*
int there_is_rect_intersection(frect_t rect1, frect_t rect2) {

   //X coordinate criteria
   if( RECT_XLEFT(rect1) >= RECT_XRIGHT(rect2) || RECT_XRIGHT(rect1) <= RECT_XLEFT(rect2) )
     return 0;

   //Y coordinate criteria
   if( RECT_YBOTTOM(rect1) <= RECT_YTOP(rect2) || RECT_YTOP(rect1) >= RECT_YBOTTOM(rect2) )
     return 0;

   return 1;
}

 int there_is_rect_intersection(rect_t rect1, rect_t rect2) {

   //X coordinate criteria
   if( RECT_XLEFT(rect1) >= RECT_XRIGHT(rect2) || RECT_XRIGHT(rect1) <= RECT_XLEFT(rect2) )
     return 0;

   //Y coordinate criteria
   if( RECT_YBOTTOM(rect1) <= RECT_YTOP(rect2) || RECT_YTOP(rect1) >= RECT_YBOTTOM(rect2) )
     return 0;

   return 1;
 }
 */

template <typename T>
bool polygon2D<T>::counterclockwisePolygon() {
    double
        x0 = points[0].x,
        y0 = points[0].y,
        x1 = points[1].x,
        y1 = points[1].y,
        x2 = points[2].x,
        y2 = points[2].y,
        cross_product = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1);

    return (cross_product > 0) ? true : false;
}

template <typename T>
polygon2D<T> *polygon2D<T>::getInversedPolygon() {
    int n_points = points.size();
    polygon2D<T> *inverse = new polygon2D(n_points);
    *inverse = *this;
    int i, j;
    for(i = 0; i < n_points; i++) {
        j = n_points - i - 1;
        inverse->points[j].x = points[i].x;
        inverse->points[j].y = points[i].y;
    }
    return inverse;
}

template <typename T>
QSharedPointer<polygon2D<T> > polygon3D<T>::makePolygon2D() {
    int i = 0, n = points.size();
    QSharedPointer<polygon2D<T> > res(new polygon2D<T>(n));

    for(i=0; i<n; i++) {
        res->points[i].x = points[i].x;
        res->points[i].y = points[i].y;
    }

    res->computeBoundingRectangle();

    return res;
}

template <typename T>
QSharedPointer< polygon2D<int> > polygon3D<T>::projectPolygon2D(perspective_matrix M) {
    int i = 0, n = points.size();
    QSharedPointer< polygon2D<int> > res(new polygon2D<int>(n));
    double X, Y;

    for(i=0; i<n; i++) {
        SceneModel::worldToImgCoords(M, points[i].x, points[i].y, points[i].z, &X, &Y);
        res->points[i].x = (int)X;
        res->points[i].y = (int)Y;
    }

    res->computeBoundingRectangle();

    return res;
}

template <typename T>
bool polygon2D<T>::pointInPolygon(T x, T y, bool strict) {
    double length;
    double x2, y2;
    double angle;
    int i, check, test_ok, nbi, n = points.size();
    point2D<T> inter, *begin, *end;

    // First case: is the point outside the bounding box ?
    Rectangle<T> bounding_rect;
    computeBoundingRectangle(bounding_rect);
    if (!bounding_rect.pointInRectangle(x, y))
        return false;

    // Or it is on the frontier of the polygon
    for (i = 0; i < n; i++) {
        begin = &points[i];
        end = &points[i < n - 1 ? i + 1 : 0];
        if (point2D<T>::pointInSegmentStrict(x, y, begin, end))
                return (strict ? 0 : 1);
    }

    // compute an acceptable size for the test segment
    length = (RECT_WIDTH(&bounding_rect) < RECT_HEIGHT(&bounding_rect) ?
              RECT_HEIGHT(&bounding_rect) : RECT_WIDTH(&bounding_rect));

    // And compute the test segment
    angle = 0.0;
    do {
        nbi = 0;
        test_ok = 1;
        x2 = x + length * cos(angle);
        y2 = y + length * sin(angle);
        for (i = 0; i < n; i++) {
            begin = &points[i];
            end = &points[i < n - 1 ? i + 1 : 0];

            if( (check = point2D<T>::isLinesIntersection(x, y, x2, y2,
                                                         POINT_2D_X(begin),POINT_2D_Y(begin),
                                                         POINT_2D_X(end), POINT_2D_Y(end), &inter)) )
                nbi++;

            // If the intersection point is too close from one of the endings
            // of the current segment, then slightly change the angle, and do
            // it again
            if (check > 0) {
                if (point2D<T>::distance(begin, &inter) < 0.01 || point2D<T>::distance(end, &inter) < 0.01) {
                    test_ok = 0;
                    angle += 0.2;
                }
            }
        }
    } while(!test_ok);

    // If the test is ok, then the point is inside the polygon
    // if we have an odd number of intersection
    if (nbi % 2 == 1)
        return true;
    else
        return false;
}

template <typename T>
QSharedPointer<polygon2D<int> > polygon2D<T>::projectImagePolygon2D(homography_matrix H) {
    int i = 0, n = points.size();
    polygon2D<int> *r = new polygon2D<int>(n);
    QSharedPointer< polygon2D<int> > res(r);
    double X, Y;
    for(i=0; i<n; i++) {
        SceneModel::homographyToImgCoords(H, points[i].x, points[i].y, &X, &Y);
        res->points[i].x = (int)X;
        res->points[i].y = (int)Y;
    }

    res->computeBoundingRectangle();

    return res;

}

template <typename T>
bool point2D<T>::pointInSegmentStrict(T x, T y, point2D<T> *begin, point2D<T> *end) {
    point2D<T> point;
    if(pointInSegment(x, y, begin, end))
        if(    (    ( x >= POINT_2D_X(begin) && x <= POINT_2D_X(end)   )
                 || ( x >= POINT_2D_X(end)   && x <= POINT_2D_X(begin) ) )
            && (    ( y >= POINT_2D_Y(end)   && y <= POINT_2D_Y(begin) )
                 || (y >= POINT_2D_Y(end)    && y <= POINT_2D_Y(begin) ) ) )
            return true;

    point.x = x;
    point.y = y;
    // if the distance between (x,y) and segment is very short. We consider that the point
    // belonsg to the segment.
    //Distance will be just taken to avoid float error
    //if(point_segment_distance(&point, begin_p, end_p)< (sqrt(2.0)/2.0))
    if(point.pointSegmentDistance(begin, end) < MAX_FLOAT_ERROR)
        return true;
    return false;
}

template <typename T>
double point2D<T>::pointSegmentDistance(point2D<T> *begin, point2D<T> *end) {
    double res = 0;
    double pscal = 0;

    pscal = (POINT_2D_X(end) - POINT_2D_X(begin)) * (x - POINT_2D_X(begin))
          + (POINT_2D_Y(end) - POINT_2D_Y(begin)) * (y - POINT_2D_Y(begin));

    if (pscal <= 0)
        res = distance(this, begin);
    else {
        pscal = (POINT_2D_X(begin) - POINT_2D_X(end)) * (x - POINT_2D_X(end))
              + (POINT_2D_Y(begin) - POINT_2D_Y(end)) * (y - POINT_2D_Y(end));
        if (pscal <= 0)
            res = distance(this, end);
        else
            res = pointLineDistance(begin, end);
    }

    return res;
}

/*template <typename T>
double point2D<T>::distance2D(point2D<T> *pt1, point2D<T> *pt2) {
    double dx = POINT_2D_X(pt1) - POINT_2D_X(pt2);
    double dy = POINT_2D_Y(pt1) - POINT_2D_Y(pt2);
    double squared_dist = dx * dx + dy * dy;

    return sqrt(fabs(squared_dist));
}*/

template <typename T>
double point2D<T>::pointLineDistance(point2D<T> *lpt1, point2D<T> *lpt2) {
    double a = POINT_2D_Y(lpt1) - POINT_2D_Y(lpt2);
    double b = POINT_2D_X(lpt2) - POINT_2D_X(lpt1);
    double c = - (a * POINT_2D_X(lpt1) + b * POINT_2D_Y(lpt1));
    double s = sqrt(a * a + b * b);

    return fabs((a * x + b * y + c) / s);
}

template <typename T>
bool point2D<T>::pointInSegment(double x, double y, point2D<T> *begin, point2D<T> *end) {
    double seg_vect[2], vect[2];
    double norm, cp;

    // first tests: is the point equal to one of the endings ?
    if (((x == POINT_2D_X(begin)) && (y == POINT_2D_Y(begin))) ||
        ((x == POINT_2D_X(end))   && (y == POINT_2D_Y(end))))
            return 1;

    seg_vect[0] = POINT_2D_X(end) - POINT_2D_X(begin);
    seg_vect[1] = POINT_2D_Y(end) - POINT_2D_Y(begin);
    norm = sqrt((seg_vect[0] * seg_vect[0]) + (seg_vect[1] * seg_vect[1]));
    seg_vect[0] = seg_vect[0] / norm;
    seg_vect[1] = seg_vect[1] / norm;

    vect[0] = x - POINT_2D_X(begin);
    vect[1] = y - POINT_2D_Y(begin);
    norm = sqrt((vect[0] * vect[0]) + (vect[1] * vect[1]));
    vect[0] = vect[0] / norm;
    vect[1] = vect[1] / norm;

    cp = (vect[0] * seg_vect[1]) - (vect[1] * seg_vect[0]);
    if (fabs(cp) > 0.01)
        return false;

    norm = 100.0;
    if (fabs(seg_vect[0]) >= fabs(seg_vect[1]) && fabs(seg_vect[0]) > 0.01)
        norm = vect[0] / seg_vect[0];
    if (fabs(seg_vect[1]) >= fabs(seg_vect[0]) && fabs(seg_vect[1]) > 0.01)
        norm = vect[1] / seg_vect[1];
    if ((norm >= 0.0) && (norm <= 1.0))
        return true;
    return false;
}

// The algorithm is described on Robert Sedgewick book, it's
// Algorithms in C++. Chap 24, Elementary Geometric Methods
// pp346--357
// If both endpoints of each line are on different "sides"
// (have different counterclockwise values) of the other,
// then the lines must intersect
template <typename T>
bool point2D<T>::isLinesIntersection(double x1, double y1, double x2, double y2,
                                    double x3, double y3, double x4, double y4,
                                    point2D<T> *pIntersection) {
    // Intersection
    if (    isCCW(x1,y1,x2,y2,x3,y3) * isCCW(x1,y1,x2,y2,x4,y4) <= 0
         && isCCW(x3,y3,x4,y4,x1,y1) * isCCW(x3,y3,x4,y4,x2,y2) <= 0 ) {
        // Process intersection point, if exists
        if (processIntersectionLines(x1, y1, x2, y2, x3, y3, x4, y4, pIntersection))
            return true;
        else
            return false;
    }
    return false;
}


template <typename T>
bool point2D<T>::processIntersectionLines(double x1, double y1, double x2, double y2,
                                       double x3, double y3, double x4, double y4,
                                       point2D<T> *pIntersection) {
    if(pIntersection == NULL) {
        std::cout << "point2D: Error: processIntersectionLines - intersection is NULL." << std::endl;
        return false;
    }

    // Same points
    if(x1==x3 && y1==y3) {
        POINT_2D_X(pIntersection) = x1;
        POINT_2D_Y(pIntersection) = y1;
        return true;
    }

    if(x1==x4 && y1==y4) {
        POINT_2D_X(pIntersection) = x1;
        POINT_2D_Y(pIntersection) = y1;
        return true;
    }

    if(x2==x3 && y2==y3) {
        POINT_2D_X(pIntersection) = x2;
        POINT_2D_Y(pIntersection) = y2;
        return true;
    }

    if(x2==x4 && y2==y4) {
        POINT_2D_X(pIntersection) = x2;
        POINT_2D_Y(pIntersection) = y2;
        return true;
    }

    // All the points have the same y
    if(y1==y2 && y2==y3 && y3==y4) {
        double max1 = std::max(x1,x2);
        double min1 = std::min(x1,x2);
        double max2 = std::max(x3,x4);
        double min2 = std::min(x3,x4);

        // no intersection
        if(max1 < min2 || max2 < min1)
            return false;

        // partial or total overlapping
        if(max1 > min2 || max2 > min1) {
            // Partial overlapping
            if(max2 > max1 && min2 > min1) {
                // The segment [min2;max1] is the overlapped part
                POINT_2D_X(pIntersection) = (min2 + max1)/2.0;
                POINT_2D_Y(pIntersection) = y3;
                return true;
            }

            if(max1 > max2 && min1 > min2) {
                // The segment [min1;max2] is the overlapped part
                POINT_2D_X(pIntersection) = (min1 + max2)/2.0;
                POINT_2D_Y(pIntersection) = y3;
                return true;
            }

            // Total overlapping
            if (min2 > min1 && max2 < max1) {
                // The segment [min2;max2] is the overlapped part
                POINT_2D_X(pIntersection) = (min2 + max2)/2.0;
                POINT_2D_Y(pIntersection) = y3;
                return true;
            }

            if (min2 < min1 && max2 > max1) {
                // The segment [min1;max1] is the overlapped part
                POINT_2D_X(pIntersection) = (min1 + max1)/2.0;
                POINT_2D_Y(pIntersection) = y3;
                return true;
            }
        }
    }

    // All the points have the same x
    if(x1==x2 && x2==x3 && x3==x4) {
        double max1 = std::max(y1,y2);
        double min1 = std::min(y1,y2);
        double max2 = std::max(y3,y4);
        double min2 = std::min(y3,y4);

        // no intersection
        if (max1 < min2 || max2 < min1)
            return false;

        // partial or total overlapping
        if(max1 > min2 || max2 > min1) {
            // Partial overlapping
            if(max2 > max1 && min2 > min1) {
                // The segment [min2;max1] is the overlapped part
                POINT_2D_Y(pIntersection) = (min2 + max1)/2.0;
                POINT_2D_X(pIntersection) = x3;
                return true;
            }

            if(max1 > max2 && min1 > min2) {
                // The segment [min1;max2] is the overlapped part
                POINT_2D_Y(pIntersection) = (min1 + max2)/2.0;
                POINT_2D_X(pIntersection) = x3;
                return true;
            }

            // Total overlapping
            if(min2 > min1 && max2 < max1) {
                // The segment [min2;max2] is the overlapped part
                POINT_2D_Y(pIntersection) = (min2 + max2)/2.0;
                POINT_2D_X(pIntersection) = x3;
                return true;
            }

            if(min2 < min1 && max2 > max1) {
                // The segment [min1;max1] is the overlapped part
                POINT_2D_Y(pIntersection) = (min1 + max1)/2.0;
                POINT_2D_X(pIntersection) = x3;
                return true;
            }
        }
    }

    // General case
    // Init
    POINT_2D_X(pIntersection) = 0;
    POINT_2D_Y(pIntersection) = 0;

    // Coordinates of the first segment
    double x12 = x2 - x1;
    double y12 = y2 - y1;

    // Coordinates of the second segment
    double x34 = x4 - x3;
    double y34 = y4 - y3;

    double x31 = x1 - x3;
    double y31 = y1 - y3;

    // Process cross-product
    double alpha = x31 * y34 - y31 * x34;
    double delta = y12 * x34 - x12 * y34;

    // Test
    if (fabs(delta) < 1e-6) {
        std::cout << "Error: point2D: processIntersectionsLines - delta is NULL" << std::endl;
        return false;
    }

    if(delta != 0) {
        alpha /= delta;
        POINT_2D_X(pIntersection) = x1 + alpha*x12;
        POINT_2D_Y(pIntersection) = y1 + alpha*y12;
    }

    return true;
}

// The algorithm is described on Robert Sedgewick book, it's
// Algorithms in C++. Chap 24, Elementary Geometric Methods
// pp346--357
// This function determine if we turn counterclockwise in
// travelling from the first to the second to the third point
// It's based on the slope comparisons
template <typename T>
bool point2D<T>::isCCWPts(point2D<T> *pPt0, point2D<T> *pPt1, point2D<T> *pPt2) {
    return isCCW( POINT_2D_X(pPt0), POINT_2D_Y(pPt0),
                  POINT_2D_X(pPt1), POINT_2D_Y(pPt1),
                  POINT_2D_X(pPt2), POINT_2D_Y(pPt2) );
}

template <typename T>
bool point2D<T>::isCCW(double x0, double y0, double x1, double y1, double x2, double y2) {
    double dx1 = x1 - x0;
    double dy1 = y1 - y0;
    double dx2 = x2 - x0;
    double dy2 = y2 - y0;

    if ((dx1*dy2) > (dy1*dx2))
            return 1;
    if ((dx1*dy2) < (dy1*dx2))
            return -1;
    if ((dx1*dx2) < 0 || (dy1*dy2) < 0)
            return -1;
    if ( (dx1*dx1+dy1*dy1) < (dx2*dx2+dy2*dy2))
            return 1;

    return 0;
}

template <typename T>
polygon3D<T>::polygon3D() {}

template <typename T>
polygon3D<T>& polygon3D<T>::operator=(polygon3D<T>& p) {
    memcpy(this, &p, sizeof(polygon3D<T>));
    points = p.points;
    return *this;
}

template <typename T>
polygon3D<T>::polygon3D(int reserve) {
    points.reserve(reserve);
}

template <typename T>
polygon3D<T>::~polygon3D() {}

template <typename T>
Rectangle<T>::Rectangle() { }

template <typename T>
Rectangle<T>::Rectangle(T x, T y, T w, T h) {
    initRectangle(x, y, w, h);
}

template <typename T>
Rectangle<T>::~Rectangle() { }

template <typename T>
void Rectangle<T>::initRectangle(T x, T y, T w, T h) {
    xleft = x;
    xright = (w > 0 ? x + w - 1 : x);
    ytop = y;
    ybottom = (h > 0 ? y + h - 1 : y);
    width = w;
    height = h;
}

template <typename T>
Rectangle<T> *Rectangle<T>::copyRectangle() {
    Rectangle<T> *copy = new Rectangle<T>();
    memcpy(copy, this, sizeof(Rectangle<T>));
    return copy;
}

template <typename T>
Rectangle<T> *Rectangle<T>::mergeRectangles(Rectangle<T> *r1, Rectangle<T> *r2) {

    if(r1 == NULL && r2 == NULL)
        return NULL;
    if(r1 == NULL)
        return r2->copyRectangle();
    if(r2 == NULL)
        return r1->copyRectangle();

    Rectangle<T> *merge = new Rectangle<T>();

    merge->xleft   = std::min(r1->xleft, r2->xleft);
    merge->xright  = std::max(r2->xright, r2->xright);
    merge->ytop    = std::min(r1->ytop, r2->ytop);
    merge->ybottom = std::max(r1->ybottom, r2->ybottom);
    merge->width   = merge->xright - merge->xleft + 1;
    merge->height  = merge->ybottom - merge->ytop + 1;

    return merge;
}

template <typename T>
Rectangle<T> *Rectangle<T>::mergeRectangle(Rectangle<T> *r) {
    return mergeRectangles(this, r);
}

template <typename T>
void Rectangle<T>::absorbRectangle(Rectangle<T> *r) {
    if(r == NULL)
        return;

    xleft   = std::min(xleft, r->xleft);
    xright  = std::max(xright, r->xright);
    ytop    = std::min(ytop, r->ytop);
    ybottom = std::max(ybottom, r->ybottom);
    width   = xright - xleft + 1;
    height  = ybottom - ytop + 1;
}

template <typename T>
void Rectangle<T>::absorbRectangle(Rectangle<T> &r) {
    xleft   = std::min(xleft, r.xleft);
    xright  = std::max(xright, r.xright);
    ytop    = std::min(ytop, r.ytop);
    ybottom = std::max(ybottom, r.ybottom);
    width   = xright - xleft + 1;
    height  = ybottom - ytop + 1;
}



template <typename T>
T Rectangle<T>::horizontalDistance(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1->xright < r2->xleft)
        return r2->xleft - r1->xright;
    else if(r2->xright < r1->xleft)
        return r1->xleft - r2->xright;
    else
        return 0;
}

template <typename T>
T Rectangle<T>::verticalDistance(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1->ybottom < r2->ytop)
        return r2->ytop - r1->ybottom;
    else if(r2->ybottom < r1->ytop)
        return r1->ytop - r2->ybottom;
    else
        return 0;
}

template <typename T>
double Rectangle<T>::rectangleDistance(Rectangle<T> *r1, Rectangle<T> *r2) {
    Rectangle<T> *inter;

    if((inter = rectanglesIntersection(r1, r2)) != NULL) {
        delete inter;
        return 0.0;
    }

    double dy = 0.0, dx = 0.0;


    if(r1->xright <= r2->xleft) {
        dx = r2->xleft - r1->xright;
    } else if(r2->xright <= r1->xleft) {
        dx = r1->xleft - r2->xright;
    }

    if(r1->ybottom <= r2->ytop) {
        dy = r2->ytop - r1->ybottom;
    } else if(r2->ybottom <= r1->ytop) {
        dy = r1->ytop - r2->ybottom;
    }

    if(dy == 0.0)
        return dx;
    else if(dx == 0.0)
        return dy;

    return sqrt(dx*dx + dy*dy);

}

template <typename T>
double Rectangle<T>::rectangleDistance(Rectangle<T> *r) {
    return rectangleDistance(this, r);
}

template <typename T>
double Rectangle<T>::rectangleGravityDistance(Rectangle<T> *r1, Rectangle<T> *r2) {
    if( r1 == NULL || r2 == NULL )
        return 0.0;

    double
        X1 = r1->xleft + r1->width / 2.0,
        X2 = r2->xleft + r2->width / 2.0,
        Y1 = r1->ytop + r1->height / 2.0,
        Y2 = r2->ytop + r2->height / 2.0;

    if(X1 == X2)
        return fabs(Y1 - Y2);
    if(Y1 == Y2)
        return fabs(X1 - X2);

    double dX = X1 - X2, dY = Y1 - Y2;

    return sqrt(dX*dX + dY*dY);
}

template <typename T>
double Rectangle<T>::rectangleGravityDistance(Rectangle<T> *r) {
    return rectangleGravityDistance(this, r);
}

template <typename T>
Rectangle<T> *Rectangle<T>::rectanglesIntersection(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1 == NULL || r2 == NULL)
        return NULL;

    int
        x_left   = std::max(r1->xleft, r2->xleft),
        x_right  = std::min(r1->xright, r2->xright),
        y_top    = std::max(r1->ytop, r2->ytop),
        y_bottom = std::min(r1->ybottom, r2->ybottom);

    if(x_left < x_right && y_top < y_bottom)
        return new Rectangle<T>(x_left, y_top, x_right - x_left + 1, y_bottom - y_top + 1);
    else
        return NULL;
}

template <typename T>
bool Rectangle<T>::rectanglesIntersection(Rectangle<T> *r1, Rectangle<T> *r2, Rectangle<T> *inter) {
    if(r1 == NULL || r2 == NULL || inter == NULL)
        return false;

    int
        x_left   = std::max(r1->xleft, r2->xleft),
        x_right  = std::min(r1->xright, r2->xright),
        y_top    = std::max(r1->ytop, r2->ytop),
        y_bottom = std::min(r1->ybottom, r2->ybottom);

    if(x_left < x_right && y_top < y_bottom) {
        inter->initRectangle(x_left, y_top, x_right - x_left + 1, y_bottom - y_top + 1);
        return true;
    }

    return false;
}


template <typename T>
Rectangle<T> *Rectangle<T>::rectangleIntersection(Rectangle<T> *r) {
    return rectanglesIntersection(this, r);
}

template <typename T>
Rectangle<T> *Rectangle<T>::rectanglesUnion(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1 == NULL && r2 == NULL)
        return NULL;
    if(r1 == NULL)
        return r2->copyRectangle();
    if(r2 == NULL)
        return r1->copyRectangle();
    int
        x_left   = std::min(r1->xleft, r2->xleft),
        x_right  = std::max(r1->xright, r2->xright),
        y_top    = std::min(r1->ytop, r2->ytop),
        y_bottom = std::max(r1->ybottom, r2->ybottom);

    return new Rectangle<T>(x_left, y_top, x_right - x_left + 1, y_bottom - y_top + 1);

}


template <typename T>
Rectangle<T> *Rectangle<T>::rectangleUnion(Rectangle<T> *r) {
    return rectanglesUnion(this, r);
}

template <typename T>
double Rectangle<T>::rectangleIntersectRatioStrict(Rectangle<T> *r1, Rectangle<T> *r2) {

    if(r1 == NULL || r2 == NULL)
        return 0.0;

    Rectangle<T> *inter = rectanglesIntersection(r1, r2);
    if(inter == NULL)
        return 0.0;

    double
        a1 = r1->width*r1->height,
        a2 = r2->width*r2->height,
        ai = inter->width*inter->height, ratio;

    if(a2 > a1)
        ratio = (a2 - ai)/a2;
    else
        ratio = (a1 - ai)/a1;

  delete inter;

  return ratio;
}


template <typename T>
double Rectangle<T>::rectangleIntersectRatioStrict(Rectangle<T> *r) {
    return rectangleIntersectRatioStrict(this, r);
}

template <typename T>
double Rectangle<T>::rectangleIntersectRatio(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1 == NULL || r2 == NULL)
        return 0.0;

    Rectangle<T> *inter = rectanglesIntersection(r1, r2);
    if(inter == NULL)
        return 0.0;

    double a1 = r1->width*r1->height, r;
    r = inter->width * inter->height / a1;

    delete inter;

    return r;
}

template <typename T>
double Rectangle<T>::rectangleIntersectRatio(Rectangle<T> *r) {
    return rectangleIntersectRatio(this, r);
}


template <typename T>
bool Rectangle<T>::rectangleInRectangle(Rectangle<T> *r1, Rectangle<T> *r2) {
    return    r1->xleft >= r2->xleft && r1->xright  >= r2->xright
           && r1->ytop  >= r2->ytop  && r1->ybottom <= r2->ybottom;
}

template <typename T>
bool Rectangle<T>::rectangleInRectangle(Rectangle<T> *r) {
    return rectangleInRectangle(this, r);
}

template <typename T>
double Rectangle<T>::rectangleIntersectionArea(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(r1 == NULL || r2 == NULL)
        return 0.0;

    Rectangle<T> *inter = rectanglesIntersection(r1, r2);
    if(inter == NULL)
        return 0.0;

    double a = inter->width*inter->height;

    delete inter;

    return a;
}

template <typename T>
double Rectangle<T>::rectangleIntersectionArea(Rectangle<T> *r) {
    return rectangleIntersectionArea(this, r);
}

template <typename T>
double Rectangle<T>::rectangleNIntersectionArea(const std::vector<Rectangle<T> *>& rectangles) {

    if(rectangles.empty())
        return 0.0;

    typename std::vector<Rectangle<T> *>::iterator r_iter = rectangles.begin(),
                                                   r_end = rectangles.end();
    double
        xLeft =   (*r_iter)->xleft,
        xRight =  (*r_iter)->xright,
        yTop =    (*r_iter)->ytop,
        yBottom = (*r_iter)->ybottom;
        r_iter++;

    for (; r_iter != r_end; r_iter++) {
        xLeft   = std::max(xLeft,  (*r_iter)->xleft);
        xRight  = std::min(xRight, (*r_iter)->xright);
        yTop    = std::max(yTop,   (*r_iter)->ytop);
        yBottom = std::min(yBottom,   (*r_iter)->ybottom);
    }

    if( xLeft <= xRight && yTop <= yBottom )
        return (xRight - xLeft + 1)*(yBottom - yTop + 1);

    return 0.0;

}


template <typename T>
double Rectangle<T>::rectangleArea() {
    return width*height;
}

template <typename T>
int Rectangle<T>::getPositionRelativeToCamera(SceneModel *smodel) {

    double x, y, X1, Y1, X2, Y2;

    //BOTTOM LEFT TEST
    X1 = xleft; Y1 = ybottom;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X1, Y1, 0.0, &x, &y);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x, y, smodel->m_OneMeterRepresentation, &X2, &Y2);
    if(Y2 <= Y1) { //Cases 0-1-2
        if(X2 >= X1) //If left point goes to right, everyone goes --> CASE 2: TOP-RIGHT
            return 2;
        //Cases 0-1
        //BOTTOM RIGHT TEST
        X1 = xright;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X1, Y1, 0.0, &x, &y);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x, y, smodel->m_OneMeterRepresentation, &X2, &Y2);
        if(X2 <= X1)  //If right point also goes to left, everyone goes --> CASE 0: TOP-LEFT
            return 0;
        //CASE 1: TOP-CENTER
        return 1;
    }

    //TOP LEFT TEST
    Y1 = ytop;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X1, Y1, 0.0, &x, &y);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x, y, smodel->m_OneMeterRepresentation, &X2, &Y2);
    if(Y2 <= Y1) { //Cases 3-4-5
        if(X2 >= X1) //If left point goes to right, everyone goes --> CASE 5: MIDDLE-RIGHT
            return 5;
        //Cases 3-4
        //TOP RIGHT TEST
        X1 = xright;
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X1, Y1, 0.0, &x, &y);
        SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x, y, smodel->m_OneMeterRepresentation, &X2, &Y2);
        if(X2 <= X1)  //If right point also goes to left, everyone goes --> CASE 3: MIDDLE-LEFT
            return 3;
        //CASE 4: MIDDLE-CENTER
        return 4;
    }

    //Cases 6-7-8
    if(X2 >= X1) //If left point goes to right, everyone goes --> CASE 8: BOTTOM-RIGHT
        return 8;
    //Cases 7-8
    //TOP RIGHT TEST
    X1 = xright;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), X1, Y1, 0.0, &x, &y);
    SceneModel::worldToImgCoords(SM_CALIB_MATRIX(smodel), x, y, smodel->m_OneMeterRepresentation, &X2, &Y2);
    if(X2 <= X1)  //If right point also goes to left, everyone goes --> CASE 6: BOTTOM-LEFT
        return 6;
    //CASE 7: BOTTOM-CENTER
    return 7;
}

template <typename T>
int Rectangle<T>::getPositionRelativeToCameraOld(SceneModel *smodel) {
    double x2d = 0.0; //TODO: SM_CAMERA_X2D_FOC_POINT(smodel);
    double y2d = 0.0; //TODO: SM_CAMERA_Y2D_FOC_POINT(smodel);
    int LeftToRight/*0:Left, 1:Middle, 2:Right*/, FrontToBack/*0:Front, 1:Center, 2:Back*/;

    if(xright < x2d)
        LeftToRight = 0;
    else if(xleft > x2d)
        LeftToRight = 2;
    else
        LeftToRight = 1;

    if(ybottom < y2d)
        FrontToBack = 0;
    else if(ytop > y2d)
        FrontToBack = 2;
    else
        FrontToBack = 1;

    return LeftToRight + 3*FrontToBack;
}

template <typename T>
void Rectangle<T>::setEstimated3DPosition(SceneModel *smodel, point3D<double>& p, int position) {
    double x, y;
    if(position < 3) //from a blob in the upper positions
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), (xleft + xright) / 2.0, ybottom, 0, &x, &y);
    else if(position < 6) //from a blob in the center positions
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), (xleft + xright) / 2.0, (ybottom + ytop) / 2.0, 0, &x, &y);
    else  //from a blob in the bottom positions
        SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), (xleft + xright) / 2.0, ytop / 2.0, 0, &x, &y);

    p.x = x;
    p.y = y;
    p.z = 0.0;
}



template <typename T>
void Rectangle<T>::setPositionAtCenterBottom(SceneModel *smodel, point3D<double>& p) {
    double x, y;
    SceneModel::imgToWorldCoordsGivenHeight(SM_CALIB_MATRIX(smodel), (xleft + xright) / 2.0, ybottom, 0.0, &x, &y);

  p.x = x;
  p.y = y;
  p.z = 0.0;
}

template <typename T>
bool Rectangle<T>::isRect1InsideRect2(Rectangle<T> *r1, Rectangle<T> *r2) {
    if(    RECT_YBOTTOM(r1) <= RECT_YBOTTOM(r2) && RECT_YTOP(r1)  >= RECT_YTOP(r2)
        && RECT_XRIGHT(r1)  <= RECT_XRIGHT(r2)  && RECT_XLEFT(r1) >= RECT_XLEFT(r2) )
        return true;
    return false;
}

template <typename T>
T Rectangle<T>::overlappingArea(Rectangle<T> *r1, Rectangle<T> *r2) {
    Rectangle<T> intersection;

    if(!rectanglesIntersection(r1, r2, &intersection))
        return 0;
    return intersection.rectangleArea();
}

template <typename T>
bool Rectangle<T>::pointInRectangle(T p_x, T p_y) {
    return xleft <= p_x && p_x <= xright && ytop <= p_y && p_y <= ybottom;
}


template <typename T>
bool Rectangle<T>::thereIsIntersection(Rectangle<T> *r1, Rectangle<T> *r2) {

    //X coordinate criteria
    if( RECT_XLEFT(r1) >= RECT_XRIGHT(r2) || RECT_XRIGHT(r1) <= RECT_XLEFT(r2) )
        return false;

   //Y coordinate criteria
   if( RECT_YBOTTOM(r1) <= RECT_YTOP(r2) || RECT_YTOP(r1) >= RECT_YBOTTOM(r2) )
     return false;

   return true;
}

#endif /* _GEOMETRIC_H_*/
